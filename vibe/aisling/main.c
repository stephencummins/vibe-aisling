/* SPDX-License-Identifier: GPL-3.0-only */
/* AISLING for VIBE FM: the FM-1 firmware's engine (firmware/src/dsp, the FM-1's own source, unchanged), played
 * natively. A fork of NACHTFAHRT (FartyPants), itself after DUB-1 and AMB-1 (Charles Vestal): see docs/MANUAL.md.
 *
 * The engine renders at its own 44.1 kHz, in the FM-1's blocks of 256. At 44.1 kHz its samples go out as they are, bit
 * for bit the FM-1's; at any other rate a windowed-sinc resampler takes them to the host's.
 *
 * Two flows (VIBE_PARALLEL_AUDIO): the events hand the audio commands through one ring (one writer, one reader); only
 * the audio calls the engine's commands (so its own queue, written and drained there, never crosses a flow); the
 * drawing reads the engine's volatile state, as the FM-1's screen does. */
#include "vibe.h"
#include "vibe_gfx.h"

VIBE_PARALLEL_AUDIO

/* the engine, as the FM-1 builds it: one translation unit. Its floating point strict (the SDK builds with
 * -ffast-math, which would let the compiler reorder the engine's sums and divide by reciprocals: the sound would no
 * longer be the FM-1's). (wasm has no fused multiply-add, so nothing contracts) */
#define EN_HOST 1
#pragma clang fp reassociate(off)
#pragma clang fp reciprocal(off)
#pragma clang fp contract(off)
#include "../../firmware/src/dsp/amb.c"

/* ------------------------------------------------------------------------------------------------ commands --- */
enum { AI_FX, AI_MOVE, AI_LKEY, AI_LMIDI, AI_PLAY, AI_FREEZE, AI_PIECE, AI_SET, AI_MACRO, AI_CLOCK, AI_PANIC };
typedef struct {
    uint16_t op;
    int16_t a;
    int32_t v;
} ai_cmd;
#define AI_QN 256u
static ai_cmd ai_q[AI_QN];
static volatile uint32_t ai_qw, ai_qr;          /* the events write ai_q, then ai_qw; the audio reads to ai_qw, then ai_qr */

static void ai_post(int op, int a, int32_t v)   /* (the control flow only) */
{
    uint32_t w = ai_qw;
    if (w - ai_qr >= AI_QN)
        return;                                  /* full: dropped (the audio has stopped: the plugin's window, say) */
    ai_q[w % AI_QN].op = (uint16_t)op;
    ai_q[w % AI_QN].a = (int16_t)a;
    ai_q[w % AI_QN].v = v;
    VIBE_RELEASE();
    ai_qw = w + 1u;
}

/* ------------------------------------------------------------------------------------- the audio's own side --- */
static uint32_t ai_frames;                       /* output frames so far (the seed's entropy, as the FM-1's clock) */
static int16_t ai_apar[P_NPARAMS];               /* a piece's recipe, as the audio loads it */
static volatile uint32_t ai_gain = 2048u;        /* Q12, the FM-1's MASTER: Volume 0.5 is 2048, en_render's host's */

static void ai_piece_load(int i)                 /* ui.c's set_piece: the recipe, its scale, its loops grown afresh;
                                                  * it plays on (or rests) as it was. Tune and MIDI out are global */
{
    uint16_t pool;
    int p;
    en_piece(i, ai_apar, &pool);
    for (p = 0; p < P_NPARAMS; p++)
        if (p != P_TUNE && p != P_MIDI)
            en_set(p, ai_apar[p]);
    en_pool(pool);
    en_seq(0);
    en_seed(ai_frames);
    en_regrow();
}

static void ai_do(const ai_cmd *c)
{
    switch (c->op) {
    case AI_FX: en_fx((uint32_t)c->v); break;
    case AI_MOVE: en_move((uint32_t)c->v); break;
    case AI_LKEY: en_lead_key(c->a, (int)c->v); break;
    case AI_LMIDI: en_lead_midi(c->a, (int)c->v); break;
    case AI_PLAY:
        if (c->v) {
            en_seed(ai_frames);                  /* (as the FM-1's PLAY: some entropy, then it plays) */
            en_play(1);
        } else
            en_play(0);
        break;
    case AI_FREEZE: en_freeze((int)c->v); break;
    case AI_PIECE: ai_piece_load(c->a); break;
    case AI_SET: en_set(c->a, (int)c->v); break;
    case AI_MACRO: en_macro(c->a, (int)c->v); break;
    case AI_CLOCK: en_clock(c->a); break;
    case AI_PANIC: en_panic(); break;
    }
}

static void ai_drain(void)                       /* the commands into the engine's queue, while it has room for a piece */
{
    uint32_t w = ai_qw, r = ai_qr;
    VIBE_ACQUIRE();
    while (r != w && QN - (q_w - q_r) > P_NPARAMS + 16u) {
        ai_do(&ai_q[r % AI_QN]);
        r++;
    }
    VIBE_RELEASE();
    ai_qr = r;
}

/* the engine's MIDI out (it sends only while MIDI out is on): from the render, so from the audio flow alone */
void en_midi_out(uint32_t st, uint32_t d1, uint32_t d2) { vibe_midi_send((uint8_t)st, (uint8_t)d1, (uint8_t)d2); }

/* ------------------------------------------------------------------------------------------- the resampler --- */
#define AI_RQ 256u                               /* the engine's block: the FM-1's (main_fm1.c: REND_FRAMES(256)) */
#define AI_RN 1024u                              /* the engine's output at 44.1 kHz, a ring (a power of two) */
#define AI_TAPS 32                               /* a frame from input frames ip-15 .. ip+16 */
#define AI_HALF 16
#define AI_PH 256                                /* phases of the fraction, linear between them */
static int32_t ai_blk[2u * AI_RQ];
static float ai_fl[AI_RN], ai_fr[AI_RN];
static uint32_t ai_fw, ai_ip;                    /* frames written; the read position's whole part */
static double ai_fp, ai_step;                    /* its fraction; 44100 / the host's rate */
static int ai_pass;                              /* the host at 44.1 kHz: the engine's samples as they are */
static float ai_h[AI_PH + 1][AI_TAPS], ai_dh[AI_PH][AI_TAPS];

#define AI_PI 3.14159265358979323846
static double ai_sin(double x)                   /* (no libm): to -pi .. pi, then its series */
{
    double t, s, x2;
    int i;
    long long n = (long long)(x / (2.0 * AI_PI) + (x < 0.0 ? -0.5 : 0.5));
    x -= (double)n * 2.0 * AI_PI;
    x2 = x * x;
    t = s = x;
    for (i = 1; i < 18; i++) {
        t *= -x2 / ((2.0 * i) * (2.0 * i + 1.0));
        s += t;
    }
    return s;
}

static void ai_table(double cut)                 /* cut: the low-pass, in cycles of the input's rate (0.5: Nyquist) */
{
    int p, k;
    for (p = 0; p <= AI_PH; p++) {
        double row[AI_TAPS], sum = 0.0;
        for (k = 0; k < AI_TAPS; k++) {
            const double d = (double)p / AI_PH + (AI_HALF - 1) - k, x = 2.0 * AI_PI * cut * d;
            const double s = d == 0.0 ? 1.0 : ai_sin(x) / x;
            const double w = d <= -AI_HALF || d >= AI_HALF ? 0.0
                           : 0.42 + 0.5 * ai_sin(AI_PI * d / AI_HALF + 0.5 * AI_PI) + 0.08 * ai_sin(2.0 * AI_PI * d / AI_HALF + 0.5 * AI_PI);
            row[k] = s * w;
            sum += row[k];
        }
        for (k = 0; k < AI_TAPS; k++)
            ai_h[p][k] = (float)(row[k] / sum);  /* (each phase passes DC at unity) */
    }
    for (p = 0; p < AI_PH; p++)
        for (k = 0; k < AI_TAPS; k++)
            ai_dh[p][k] = ai_h[p + 1][k] - ai_h[p][k];
}

static void ai_render(void)                      /* one engine block into the ring */
{
    uint32_t k;
    ai_drain();
    en_render(ai_blk, AI_RQ, ai_gain);
    for (k = 0; k < AI_RQ; k++) {
        const uint32_t i = (ai_fw + k) & (AI_RN - 1u);
        ai_fl[i] = (float)ai_blk[2u * k] * (1.0f / 8388608.0f);   /* (24-bit: exact in a float) */
        ai_fr[i] = (float)ai_blk[2u * k + 1u] * (1.0f / 8388608.0f);
    }
    ai_fw += AI_RQ;
}

static float ai_clip(float x) { return x > 1.0f ? 1.0f : x < -1.0f ? -1.0f : x; }

void vibe_on_audio(const float *in, float *out_l, float *out_r, int frames)
{
    int n;
    (void)in;
    if (ai_pass) {
        for (n = 0; n < frames; n++) {
            uint32_t i;
            if (ai_ip == ai_fw)
                ai_render();
            i = ai_ip++ & (AI_RN - 1u);
            out_l[n] = ai_clip(ai_fl[i]);
            out_r[n] = ai_clip(ai_fr[i]);
        }
    } else {
        for (n = 0; n < frames; n++) {
            const double ph = ai_fp * AI_PH;
            const int p = (int)ph;
            const float t = (float)(ph - p);
            const float *h = ai_h[p], *dh = ai_dh[p];
            const uint32_t base = ai_ip - (AI_HALF - 1);
            float l = 0.0f, r = 0.0f;
            uint32_t whole;
            int k;
            while ((int32_t)(ai_fw - ai_ip) < AI_HALF + 1)
                ai_render();
            for (k = 0; k < AI_TAPS; k++) {
                const float c = h[k] + t * dh[k];
                const uint32_t i = (base + (uint32_t)k) & (AI_RN - 1u);
                l += c * ai_fl[i];
                r += c * ai_fr[i];
            }
            out_l[n] = ai_clip(l);
            out_r[n] = ai_clip(r);
            ai_fp += ai_step;
            whole = (uint32_t)ai_fp;
            ai_ip += whole;
            ai_fp -= whole;
        }
    }
    ai_frames += (uint32_t)frames;
}

/* ----------------------------------------------------------------------------------- the control flow's side --- */
static int16_t ai_par[P_NPARAMS];                /* the piece's parameters as the controls last set them */
static int ai_piece, ai_filter;                  /* the piece; the Filter action (EN_MAC_FILTER), -50 .. 50 */
static uint32_t ai_fxp, ai_fxb, ai_mv;           /* the effects held: on the pads, on the black keys; the moves */
static int ai_help;                              /* HOME's overview open */
static float ai_acc[4];                          /* the encoders' fractions of a detent */
static const char *ai_piece_ch[EN_NPIECES];

enum { AP_VOLUME, AP_PIECE, AP_N };
static const vibe_param AI_PARAMS[AP_N] = {
    {"volume", "Volume", 0.0f, 1.0f, 0.5f, "", 0, 0},
    {"piece", "Piece", 0.0f, (float)(EN_NPIECES - 1), 0.0f, "", EN_NPIECES, ai_piece_ch},
};

/* the effects grid: the pads, a row a group (the FM-1's grid, in its order: EN_FX_*). Echo, Filter, Space, Moves */
static const uint32_t AI_GROUP_RGB[4] = {0xFF8A1F, 0x3FD0E0, 0x9B6BFF, 0x1EA462};
static const char *const AI_FX_NAME[16] = {"Repeat", "Time", "Tape stop", "Shimmer", "Low-pass", "High-pass", "Sweep",
                                           "Phaser", "Reverb", "Ceo", "Freeze", "Wash", "Octave", "Return", "Fras", "Macalla"};
/* the black keys, as the FM-1's (ui.c BLACK_DO), the first ten: an effect each while held, or a move (100 + EN_MV_*) */
#define AI_NBLACK 10
static const uint8_t AI_BLACK_DO[AI_NBLACK] = {EN_FX_TAKEOFF, EN_FX_SUPERNOVA, EN_FX_TWITTER, EN_FX_REPEAT, EN_FX_SHIMMER,
                                               EN_FX_FREEZE, EN_FX_OCTAVE, EN_FX_RETURN, 100 + EN_MV_RISE, 100 + EN_MV_CLOSE};
static const int8_t AI_WHITE[12] = {0, -1, 1, -1, 2, 3, -1, 4, -1, 5, -1, 6};   /* a pitch class's white key, -1 black */
static const int8_t AI_BLACKPC[12] = {-1, 0, -1, 1, -1, -1, 2, -1, 3, -1, 4, -1};

static void ai_leds(void)
{
    int p;
    for (p = 0; p < 16; p++)
        vibe_led_pad(p, AI_GROUP_RGB[p >> 2], ai_fxp >> p & 1u ? 1.0f : 0.18f);
    vibe_led_button(VB_PLAY, 0x1EA462, en_playing ? 1.0f : 0.15f);
    vibe_led_button(VB_REC, 0xFF3B30, en_frozen ? 1.0f : 0.15f);
}

static void ai_fx_send(void)
{
    ai_post(AI_FX, 0, (int32_t)(ai_fxp | ai_fxb));
    ai_post(AI_MOVE, 0, (int32_t)ai_mv);
}

static void ai_set_piece(int i)                  /* (the actions back to 0 first, as ui.c's mac_home) */
{
    uint16_t pool;
    ai_piece = (i % EN_NPIECES + EN_NPIECES) % EN_NPIECES;
    en_piece(ai_piece, ai_par, &pool);           /* (pure: the recipe's table) */
    if (ai_filter) {
        ai_filter = 0;
        ai_post(AI_MACRO, EN_MAC_FILTER, 0);
    }
    ai_post(AI_PIECE, ai_piece, 0);
}

static void ai_key(int k, int down)              /* the 25 keys, C to C: the white the Lead's scale degrees (ui.c's
                                                  * en_lead_key), the black the FM-1's black-key effects */
{
    const int oct = k / 12, pc = k % 12;
    if (AI_WHITE[pc] >= 0)
        ai_post(AI_LKEY, oct * 7 + AI_WHITE[pc], down);
    else {
        const int b = oct * 5 + AI_BLACKPC[pc];
        int d;
        if (b < 0 || b >= AI_NBLACK)
            return;
        d = AI_BLACK_DO[b];
        if (d >= 100)
            ai_mv = down ? ai_mv | (uint32_t)(d - 100) : ai_mv & ~(uint32_t)(d - 100);
        else
            ai_fxb = down ? ai_fxb | 1u << d : ai_fxb & ~(1u << d);
        ai_fx_send();
    }
}

static void ai_pad(int p, int down)
{
    if (p < 0 || p >= 16)
        return;
    ai_fxp = down ? ai_fxp | 1u << p : ai_fxp & ~(1u << p);
    ai_fx_send();
    ai_leds();
}

static void ai_midi(const uint8_t *m)            /* (ui.c's midi_in) notes on any channel: the Lead; clock: the grid */
{
    const uint32_t st = m[0], hi = st & 0xF0u;
    if (st == 0xF8u || st == 0xFAu || st == 0xFBu || st == 0xFCu)
        ai_post(AI_CLOCK, st == 0xF8u ? EN_CLK_TICK : st == 0xFAu ? EN_CLK_START : st == 0xFBu ? EN_CLK_CONTINUE : EN_CLK_STOP, 0);
    else if (hi == 0x90u || hi == 0x80u)
        ai_post(AI_LMIDI, m[1] & 0x7F, hi == 0x90u && m[2]);
    else if (hi == 0xB0u && (m[1] == 120u || m[1] == 123u))
        ai_post(AI_LMIDI, -1, 0);
}

/* encoders 1-4 on the AISLING page: Filter, Evolve, Motion, Drift (ui.c's VIEW_KNOB[0]) */
static const int AI_HOME_KNOB[4] = {-1, P_EVOLVE, P_MOTION, P_DRIFT};
static const char *const AI_HOME_NAME[4] = {"Filter", "Evolve", "Motion", "Drift"};

static void ai_enc(int id, float turn)
{
    int steps, p, v;
    if (id < VE_1 || id > VE_4)
        return;
    ai_acc[id] += turn;
    steps = (int)ai_acc[id];
    ai_acc[id] -= (float)steps;
    if (!steps)
        return;
    p = AI_HOME_KNOB[id];
    if (p < 0) {
        v = ai_filter + steps;
        v = v < -50 ? -50 : v > 50 ? 50 : v;
        if (v != ai_filter) {
            ai_filter = v;
            ai_post(AI_MACRO, EN_MAC_FILTER, v);
        }
        return;
    }
    v = ai_par[p] + steps;
    if (v < en_param_info(p)->lo)
        v = en_param_info(p)->lo;
    if (v > en_param_info(p)->hi)
        v = en_param_info(p)->hi;
    if (v != ai_par[p]) {
        ai_par[p] = (int16_t)v;
        ai_post(AI_SET, p, v);
    }
}

static void ai_button(int b, int down)
{
    if (!down)
        return;
    switch (b) {
    case VB_PLAY: ai_post(AI_PLAY, 0, !en_playing); break;
    case VB_STOP: ai_post(AI_PLAY, 0, 0); break;
    case VB_REC: ai_post(AI_FREEZE, 0, !en_frozen); break;   /* (the FM-1's REC tapped: the dub held) */
    case VB_LEFT:
    case VB_RIGHT:
        ai_set_piece(ai_piece + (b == VB_RIGHT ? 1 : -1));
        vibe_param_set(AP_PIECE, (float)ai_piece);
        break;
    case VB_HOME: ai_help = !ai_help; break;
    case VB_RETURN: ai_help = 0; break;
    }
}

static void ai_param(int id, float v)
{
    if (id == AP_VOLUME) {
        v = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
        ai_gain = (uint32_t)(v * 4096.0f + 0.5f);
    } else if (id == AP_PIECE) {
        const int i = (int)(v + 0.5f);
        if (i != ai_piece)
            ai_set_piece(i);
    }
}

void vibe_on_event(const vibe_event *e)
{
    switch (e->type) {
    case VIBE_EV_KEY: ai_key(e->id, e->down); break;
    case VIBE_EV_PAD: ai_pad(e->id, e->down); break;
    case VIBE_EV_BUTTON: ai_button(e->id, e->down); break;
    case VIBE_EV_ENCODER: ai_enc(e->id, e->value); break;
    case VIBE_EV_MIDI: ai_midi(e->midi); break;
    case VIBE_EV_PARAM: ai_param(e->id, e->value); break;
    default: break;
    }
}

void vibe_on_init(float sample_rate)
{
    uint16_t pool;
    int i;
    ai_pass = sample_rate > 44099.5f && sample_rate < 44100.5f;
    ai_step = 44100.0 / (double)sample_rate;
    if (!ai_pass)
        ai_table(sample_rate < 44100.0f ? 0.45 * (double)sample_rate / 44100.0 : 0.45);
    ai_fw = ai_ip = 0;
    ai_fp = 0.0;
    memset(ai_fl, 0, sizeof ai_fl);
    memset(ai_fr, 0, sizeof ai_fr);
    ai_qw = ai_qr = 0;
    for (i = 0; i < EN_NPIECES; i++)
        ai_piece_ch[i] = en_piece_name(i);
    vibe_params(AI_PARAMS, AP_N);
    ai_gain = (uint32_t)(vibe_param_get(AP_VOLUME) * 4096.0f + 0.5f);
    /* (host/en_render.c's boot, call for call: the FM-1's sound from the first sample) */
    ai_piece = 0;
    ai_filter = 0;
    en_init();
    en_piece(ai_piece, ai_par, &pool);
    for (i = 0; i < P_NPARAMS; i++)
        if (i != P_TUNE && i != P_MIDI)
            en_set(i, ai_par[i]);
    en_pool(pool);
    en_regrow();
    en_play(1);
    ai_leds();
}

/* ---------------------------------------------------------------------------------------------- the display --- */
#define AI_GREEN 0x1EA462
#define AI_BG 0x0B110E
#define AI_DIM 0x6E7A74
#define AI_TEXT 0xE8EEEA
static const uint32_t AI_LAYER_RGB[5] = {0x9B6BFF, 0xFF5FA2, 0xF2C14E, 0x3FD0E0, 0x9AA0A6};
static const char *const AI_LAYER_NAME[5] = {"Drone", "Sequence", "Lead", "Seq 2", "Atmos"};

static char *ai_cat(char *d, const char *s)      /* (no libc's strcat: returns the end) */
{
    while (*s)
        *d++ = *s++;
    *d = 0;
    return d;
}

static const char *ai_chord(char *nm)            /* (ui.c's chord_now) the chord's name, "Em7" */
{
    const int slot = en_chord_slot;
    int pcs[EN_NPAD + 1], k;
    if (slot >= 0 && slot < 8 && ai_par[P_CQ0 + slot])
        return en_chord_label(ai_par[P_CR0 + slot], ai_par[P_CQ0 + slot], ai_par[P_CX0 + slot], nm);
    pcs[0] = en_chord_root % 12;
    for (k = 0; k < EN_NPAD; k++)
        pcs[k + 1] = en_chord[k] % 12;
    return en_chord_name(pcs, EN_NPAD + 1, nm);
}

static const char *const AI_HELP[] = {
    "WHITE KEYS", "play the Lead",           /* (the overview's cells hold about 22 characters) */
    "BLACK KEYS", "hold an effect, a move",
    "PADS", "hold an effect",
    "ENCODER 1", "Filter: shut / open",
    "ENCODER 2", "Evolve: change",
    "ENCODER 3", "Motion: arrangement",
    "ENCODER 4", "Drift: a chord's length",
    "< >", "the piece",
    "PLAY", "play / rest",
    "STOP", "rest (tails ring on)",
    "REC", "hold the dub",
    "MIDI IN", "Lead notes, clock",
};

static int ai_drawn;

void vibe_on_draw(uint16_t *fb)
{
    char s[48], *q;
    int i, x;
    (void)fb;
    if (!ai_drawn) {
        ai_drawn = 1;
        ai_leds();
    }
    vibe_led_button(VB_PLAY, AI_GREEN, en_playing ? 1.0f : 0.15f);
    vibe_led_button(VB_REC, 0xFF3B30, en_frozen ? 1.0f : 0.15f);
    if (ai_help) {
        vg_controls("AISLING", AI_HELP, (int)(sizeof AI_HELP / sizeof AI_HELP[0] / 2));
        return;
    }
    vg_clear(AI_BG);

    /* the header: the page's colour; the piece; its key and tempo */
    vg_rect(0, 0, 480, 36, AI_GREEN);
    vg_text("AISLING", 12, 10, VG_BOLD, 0x06140C, VG_LEFT);
    vg_text(en_piece_name(ai_piece), 240, 9, VG_MED, 0x06140C, VG_CENTER);
    q = ai_cat(s, EN_NOTE_NAME[ai_par[P_ROOT] % 12]);
    q = ai_cat(q, " ");
    q = ai_cat(q, EN_MODE_NAME[ai_par[P_MODE] % EN_NMODES]);
    q = ai_cat(q, "  ");
    vg_itoa(ai_par[P_TEMPO], q);
    vg_text(s, 468, 10, VG_BOLD, 0x06140C, VG_RIGHT);

    /* the arc: the section, its eight bars; playing or resting, the dub held */
    vg_text(EN_SEC_NAME[en_section % EN_NSEC], 12, 46, VG_BOLD, AI_TEXT, VG_LEFT);
    for (i = 0; i < 8; i++)
        vg_rect(96 + i * 18, 49, 14, 10, i == en_phrase_bar % 8u ? AI_GREEN : i < (int)(en_phrase_bar % 8u) ? 0x2F5A44 : 0x1C2621);
    vg_text(en_playing ? "PLAYING" : "RESTING", 468, 46, VG_BOLD, en_playing ? AI_GREEN : AI_DIM, VG_RIGHT);
    if (en_frozen)
        vg_text("HOLD", 380, 46, VG_BOLD, 0xFF3B30, VG_RIGHT);

    /* the chord, and how near the next */
    vg_text(ai_chord(s), 240, 74, VG_BIG | VG_X2, AI_TEXT, VG_CENTER);
    vg_rect(170, 128, 140, 3, 0x1C2621);
    vg_rect(170, 128, (int)(140.0f * (en_drift < 0.0f ? 0.0f : en_drift > 1.0f ? 1.0f : en_drift)), 3, AI_GREEN);

    /* the effects held */
    q = s;
    *q = 0;
    for (i = 0; i < 16 && q < s + 30; i++)
        if ((ai_fxp | ai_fxb) >> i & 1u) {
            if (q != s)
                q = ai_cat(q, " + ");
            q = ai_cat(q, AI_FX_NAME[i]);
        }
    for (i = 0; i < 2; i++)
        if (ai_mv >> i & 1u && q < s + 30) {
            if (q != s)
                q = ai_cat(q, " + ");
            q = ai_cat(q, i ? "Close" : "Rise");
        }
    if (*s)
        vg_text(s, 240, 138, VG_MED, 0xFF8A1F, VG_CENTER);

    /* the sound: the scope */
    {
        const uint32_t w = en_scope_w;
        float y0 = 186.0f;
        for (x = 0; x < 240; x++) {
            const float v = (float)en_scope[(w + (uint32_t)x) % EN_SCOPE_N] * (1.0f / 32768.0f);
            const float y = 186.0f - 22.0f * (v < -1.0f ? -1.0f : v > 1.0f ? 1.0f : v);
            if (x)
                vg_line((float)(x - 1) * 2.0f, y0, (float)x * 2.0f, y, 1.5f, 0x2F7A55);
            y0 = y;
        }
    }

    /* the five layers */
    for (i = 0; i < 5; i++) {
        const float a = en_layer_act[i];
        x = 12 + i * 92;
        vg_text(AI_LAYER_NAME[i], x, 206, VG_SMALL, AI_LAYER_RGB[i], VG_LEFT);
        vg_rect(x, 224, 84, 5, 0x1C2621);
        vg_rect(x, 224, (int)(84.0f * (a < 0.0f ? 0.0f : a > 1.0f ? 1.0f : a)), 5, AI_LAYER_RGB[i]);
    }

    /* encoders 1-4: their values, over F1-F6 */
    for (i = 0; i < 4; i++) {
        const int p = AI_HOME_KNOB[i];
        x = i * 120;
        vg_rect(x + 4, 238, 112, 42, 0x131C17);
        vg_text(AI_HOME_NAME[i], x + 60, 241, VG_SMALL, AI_DIM, VG_CENTER);
        if (p < 0) {
            if (!ai_filter)
                ai_cat(s, "Off");
            else {
                q = ai_cat(s, ai_filter < 0 ? "Shut " : "Open ");
                vg_itoa((ai_filter < 0 ? -ai_filter : ai_filter) * 2, q);
            }
        } else
            en_param_text(p, ai_par[p], s);
        vg_text(s, x + 60, 256, VG_MED, AI_TEXT, VG_CENTER);
    }
}
