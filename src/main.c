/*
 * LED lampa za postolje v2 - STM8S103F3P6, registarski nivo (SDCC)
 *
 *  Traka preko 3x BC337 (HIGH = svetli), 5 V iz MT3608, hardverski PWM:
 *      R -> PC3 (TIM1_CH3)   G -> PC4 (TIM1_CH4)   B -> PA3 (TIM2_CH3)
 *  Touch TTP223:          PD4   (HIGH = dodir)
 *  Displej TM1637:        PA1 = CLK, PA2 = DIO
 *  Merenje baterije:      PD3 (AIN4) - cvor 10k (sa +) + 1N4007 na GND
 *  Punjac TP4056:         PD5 <- CHRG (LOW = puni)  PD6 <- STDBY (LOW = puna)
 *
 *  Upravljanje:
 *      kratak dodir   - mod 1: sledeca boja (odmah)   mod 3: boja disanja
 *      drzanje 3 s    - sledeci mod  1 -> 2 -> 3 -> 4 -> 5 -> 1
 *  Modovi: 1 stalna boja, 2 duga, 3 disanje, 4 sveca, 5 smena boja
 *
 *  Displej (stalno upaljen):
 *      "1 80"  mod 1, baterija 80 %
 *      "3C90"  mod 3, puni se, oko 90 %
 *      "FULL"  napunjeno
 *      "-LO-"  prazna, traka ugasena (trepce)
 *
 *  Takt: fabricki HSI/8 = 2 MHz
 */
#include <stdint.h>

/* ===================== PODESAVANJA ===================== */
#define VDIODE_MV       560UL   /* izmeren napon na diodi (PD3 - GND)            */
#define HOLD_MS         3000u   /* koliko se drzi dodir za promenu moda          */
#define MAX_BRIGHT      200u    /* najveci sjaj trake, 0-255 (manje = duze traje) */
#define DISP_BRIGHT     1u      /* sjaj displeja 0-7                             */
#define PCT_STEP        10u     /* procenat se zaokruzuje na ovaj korak          */
#define LOAD_COMP_MV    40u     /* korekcija pada napona dok traka svetli        */
#define CHG_COMP_MV     100u    /* korekcija naduvanog napona dok se puni        */
#define TH_LOW          3450u   /* ispod: broj na displeju trepce                */
#define TH_CRIT         3300u   /* ispod: traka se gasi                          */
#define DUGA_KORAK      1u      /* brzina duge (1 = krug za ~15 s)               */
#define SMENA_TICKS     300u    /* mod 5: koliko traje jedna boja (x10 ms)       */

#define HOLD_TICKS      (HOLD_MS / 10u)
#define NMODES          5u

/* ===================== registri ===================== */
#define R8(a)       (*(volatile uint8_t *)(a))

#define PA_ODR      R8(0x5000)
#define PA_IDR      R8(0x5001)
#define PA_DDR      R8(0x5002)
#define PA_CR1      R8(0x5003)
#define PC_ODR      R8(0x500A)
#define PC_DDR      R8(0x500C)
#define PC_CR1      R8(0x500D)
#define PD_IDR      R8(0x5010)
#define PD_DDR      R8(0x5011)
#define PD_CR1      R8(0x5012)

#define FLASH_IAPSR R8(0x505F)
#define FLASH_DUKR  R8(0x5064)
#define EE_COLOR    R8(0x4000)
#define EE_MODE     R8(0x4001)

#define TIM1_CR1    R8(0x5250)
#define TIM1_EGR    R8(0x5257)
#define TIM1_CCMR3  R8(0x525A)
#define TIM1_CCMR4  R8(0x525B)
#define TIM1_CCER2  R8(0x525D)
#define TIM1_PSCRH  R8(0x5260)
#define TIM1_PSCRL  R8(0x5261)
#define TIM1_ARRH   R8(0x5262)
#define TIM1_ARRL   R8(0x5263)
#define TIM1_CCR3H  R8(0x5269)
#define TIM1_CCR3L  R8(0x526A)
#define TIM1_CCR4H  R8(0x526B)
#define TIM1_CCR4L  R8(0x526C)
#define TIM1_BKR    R8(0x526D)

/* STM8S103: TIM2 ima 2 rezervisana bajta posle CR1 */
#define TIM2_CR1    R8(0x5300)
#define TIM2_EGR    R8(0x5306)
#define TIM2_CCMR3  R8(0x5309)
#define TIM2_CCER2  R8(0x530B)
#define TIM2_PSCR   R8(0x530E)
#define TIM2_ARRH   R8(0x530F)
#define TIM2_ARRL   R8(0x5310)
#define TIM2_CCR3H  R8(0x5315)
#define TIM2_CCR3L  R8(0x5316)

#define TIM4_CR1    R8(0x5340)
#define TIM4_SR     R8(0x5344)
#define TIM4_EGR    R8(0x5345)
#define TIM4_PSCR   R8(0x5347)
#define TIM4_ARR    R8(0x5348)

#define ADC_CSR     R8(0x5400)
#define ADC_CR1     R8(0x5401)
#define ADC_CR2     R8(0x5402)
#define ADC_DRH     R8(0x5404)
#define ADC_DRL     R8(0x5405)
#define ADC_TDRL    R8(0x5407)

/* ===================== pinovi ===================== */
#define PIN_R       (1u << 3)   /* PC3 */
#define PIN_G       (1u << 4)   /* PC4 */
#define PIN_B       (1u << 3)   /* PA3 */
#define TM_CLK      (1u << 1)   /* PA1 */
#define TM_DIO      (1u << 2)   /* PA2 */
#define TOUCH_BIT   (1u << 4)   /* PD4 */
#define CHRG_BIT    (1u << 5)   /* PD5 */
#define STDBY_BIT   (1u << 6)   /* PD6 */

/* ===================== boje (mod 1) ===================== */
static const uint8_t PAL[][3] = {
    {255,   0,   0},    /*  1 crvena               */
    {255,  45,   0},    /*  2 narandzasta          */
    {255,  95,   0},    /*  3 zuckasto-narandzasta */
    {255, 170,   0},    /*  4 zuta                 */
    {140, 255,   0},    /*  5 limeta               */
    {  0, 255,   0},    /*  6 zelena               */
    {  0, 255, 110},    /*  7 tirkizna             */
    {  0, 255, 255},    /*  8 cijan                */
    {  0, 110, 255},    /*  9 nebo plava           */
    {  0,   0, 255},    /* 10 plava                */
    {110,   0, 255},    /* 11 ljubicasta           */
    {255,   0, 255},    /* 12 magenta              */
    {255,  30, 110},    /* 13 roze                 */
    {255, 140,  50},    /* 14 topla bela           */
    {255, 255, 255}     /* 15 hladna bela          */
};
#define NCOLORS     ((uint8_t)(sizeof(PAL) / 3))

/* napon (mV) za 0, 10, 20 ... 100 % (LiPo, malo opterecenje) */
static const uint16_t PCT_MV[11] = {
    3300, 3690, 3730, 3770, 3800, 3840, 3870, 3950, 4020, 4110, 4200
};

/* 7-segmentne cifre i slova */
static const uint8_t DIG[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
#define SEG_C       0x39
#define SEG_F       0x71
#define SEG_U       0x3E
#define SEG_L       0x38
#define SEG_O       0x3F
#define SEG_MINUS   0x40
#define SEG_BLANK   0x00

/* ===================== tajmer za vreme (1 ms) ===================== */
static void tim4_init(void)
{
    TIM4_PSCR = 4;              /* 2 MHz / 16 = 125 kHz */
    TIM4_ARR  = 124;            /* 125 taktova = 1 ms   */
    TIM4_EGR  = 1;
    TIM4_SR   = 0;
    TIM4_CR1  = 1;
}

static void wait_ms(uint8_t n)
{
    while (n--) {
        while (!(TIM4_SR & 1)) ;
        TIM4_SR = 0;
    }
}

/* ===================== PWM za traku (~7,8 kHz, 8 bita) ===================== */
static void pwm_init(void)
{
    /* pinovi kao izlazi sa nulom dok tajmeri ne preuzmu */
    PC_ODR &= (uint8_t)~(PIN_R | PIN_G);
    PC_DDR |= (PIN_R | PIN_G);
    PC_CR1 |= (PIN_R | PIN_G);
    PA_ODR &= (uint8_t)~PIN_B;
    PA_DDR |= PIN_B;
    PA_CR1 |= PIN_B;

    /* TIM1: kanali 3 i 4 */
    TIM1_PSCRH = 0;  TIM1_PSCRL = 0;
    TIM1_ARRH  = 0;  TIM1_ARRL  = 255;
    TIM1_CCMR3 = 0x68;          /* PWM mod 1 + preload */
    TIM1_CCMR4 = 0x68;
    TIM1_CCR3H = 0;  TIM1_CCR3L = 0;
    TIM1_CCR4H = 0;  TIM1_CCR4L = 0;
    TIM1_CCER2 = (1u << 0) | (1u << 4);     /* CC3E, CC4E */
    TIM1_EGR   = 1;
    TIM1_BKR   = 0x80;          /* MOE: TIM1 izlazi ukljuceni */
    TIM1_CR1   = 0x81;          /* ARPE + CEN */

    /* TIM2: kanal 3 */
    TIM2_PSCR  = 0;
    TIM2_ARRH  = 0;  TIM2_ARRL  = 255;
    TIM2_CCMR3 = 0x68;
    TIM2_CCR3H = 0;  TIM2_CCR3L = 0;
    TIM2_CCER2 = (1u << 0);     /* CC3E */
    TIM2_EGR   = 1;
    TIM2_CR1   = 0x81;
}

static void strip_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    TIM1_CCR3L = (uint8_t)(((uint16_t)r * MAX_BRIGHT) >> 8);
    TIM1_CCR4L = (uint8_t)(((uint16_t)g * MAX_BRIGHT) >> 8);
    TIM2_CCR3L = (uint8_t)(((uint16_t)b * MAX_BRIGHT) >> 8);
}

/* ===================== TM1637 (2 zice, bit-bang) ===================== */
/* "1" = pin pusten (pull-up na modulu), "0" = pin vucen na masu */
static void tm_delay(void) { volatile uint8_t i; for (i = 0; i < 4; i++) ; }
#define CLK_HI()  (PA_DDR &= (uint8_t)~TM_CLK)
#define CLK_LO()  (PA_DDR |= TM_CLK)
#define DIO_HI()  (PA_DDR &= (uint8_t)~TM_DIO)
#define DIO_LO()  (PA_DDR |= TM_DIO)

static void tm_start(void)
{
    DIO_HI(); CLK_HI(); tm_delay();
    DIO_LO(); tm_delay();
    CLK_LO(); tm_delay();
}

static void tm_stop(void)
{
    CLK_LO(); DIO_LO(); tm_delay();
    CLK_HI(); tm_delay();
    DIO_HI(); tm_delay();
}

static void tm_byte(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {           /* LSB prvi */
        CLK_LO();
        if (b & 1) DIO_HI(); else DIO_LO();
        tm_delay();
        CLK_HI(); tm_delay();
        b >>= 1;
    }
    CLK_LO(); DIO_HI(); tm_delay();     /* ACK takt (ne proveravamo) */
    CLK_HI(); tm_delay();
    CLK_LO(); tm_delay();
}

static void tm_init(void)
{
    PA_ODR &= (uint8_t)~(TM_CLK | TM_DIO);  /* kad su izlaz, uvek 0 */
    PA_CR1 |= (TM_CLK | TM_DIO);            /* kao ulaz: pull-up    */
    PA_DDR &= (uint8_t)~(TM_CLK | TM_DIO);  /* pusteni = 1          */
}

static uint8_t tm_cache[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static void tm_show(const uint8_t *seg, uint8_t on)
{
    uint8_t i, same = (tm_cache[4] == on);
    for (i = 0; i < 4; i++) if (tm_cache[i] != seg[i]) same = 0;
    if (same) return;                   /* nista se nije promenilo */
    for (i = 0; i < 4; i++) tm_cache[i] = seg[i];
    tm_cache[4] = on;

    tm_start(); tm_byte(0x40); tm_stop();               /* upis, auto-adresa */
    tm_start(); tm_byte(0xC0);                          /* od cifre 0        */
    for (i = 0; i < 4; i++) tm_byte(seg[i]);
    tm_stop();
    tm_start(); tm_byte(on ? (uint8_t)(0x88 | DISP_BRIGHT) : 0x80); tm_stop();
}

/* ===================== baterija ===================== */
static uint16_t vbat_mv(void)
{
    uint32_t sum = 0;
    uint8_t i, lo, hi;
    volatile uint8_t d;

    ADC_TDRL = (1u << 4);
    ADC_CSR  = 4;                       /* AIN4 = PD3 */
    ADC_CR2  = (1u << 3);               /* desno poravnanje */
    ADC_CR1  = (1u << 4) | 1;
    for (d = 0; d < 50; d++) ;

    for (i = 0; i < 16; i++) {
        ADC_CR1 |= 1;
        while (!(ADC_CSR & 0x80)) ;
        lo = ADC_DRL;
        hi = ADC_DRH;
        ADC_CSR &= (uint8_t)~0x80;
        sum += ((uint16_t)hi << 8) | lo;
    }
    ADC_CR1 &= (uint8_t)~1;

    if (sum == 0) return 0;
    return (uint16_t)((VDIODE_MV * 1023UL * 16UL) / sum);
}

static uint8_t pct_from_mv(uint16_t v)
{
    uint8_t i, p;
    if (v <= PCT_MV[0])  return 0;
    if (v >= PCT_MV[10]) return 100;
    for (i = 0; i < 10; i++)
        if (v < PCT_MV[i + 1]) break;
    p = (uint8_t)(i * 10u + (uint16_t)(v - PCT_MV[i]) * 10u / (PCT_MV[i + 1] - PCT_MV[i]));
    p = (uint8_t)(((p + PCT_STEP / 2u) / PCT_STEP) * PCT_STEP);
    return p > 100 ? 100 : p;
}

/* ===================== EEPROM ===================== */
static void ee_write(volatile uint8_t *addr, uint8_t v)
{
    if (*addr == v) return;
    FLASH_DUKR = 0xAE;
    FLASH_DUKR = 0x56;
    while (!(FLASH_IAPSR & (1u << 3))) ;   /* DUL */
    *addr = v;
    while (!(FLASH_IAPSR & (1u << 2))) ;   /* EOP */
    FLASH_IAPSR &= (uint8_t)~(1u << 3);
}

/* ===================== efekti ===================== */
static uint8_t R, G, B;

static void wheel(uint16_t h)           /* h = 0..1535, puna zasicenost */
{
    uint8_t f = (uint8_t)(h & 0xFF);
    switch ((uint8_t)(h >> 8)) {
        case 0:  R = 255;           G = f;             B = 0;             break;
        case 1:  R = (uint8_t)(255 - f); G = 255;      B = 0;             break;
        case 2:  R = 0;             G = 255;           B = f;             break;
        case 3:  R = 0;             G = (uint8_t)(255 - f); B = 255;      break;
        case 4:  R = f;             G = 0;             B = 255;           break;
        default: R = 255;           G = 0;             B = (uint8_t)(255 - f); break;
    }
}

static uint8_t mix(uint8_t a, uint8_t b, uint8_t f)    /* a -> b, f = 0..255 */
{
    int16_t d = (int16_t)b - (int16_t)a;
    return (uint8_t)((int16_t)a + (int16_t)((d * (int16_t)f) / 255));
}

static uint16_t lfsr = 0xACE1u;
static uint8_t rnd(void)
{
    uint8_t bit = (uint8_t)(((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u);
    lfsr = (uint16_t)((lfsr >> 1) | ((uint16_t)bit << 15));
    return (uint8_t)lfsr;
}

/* ===================== main ===================== */
int main(void)
{
    uint16_t t = 0, t_meas = 0, t_change = 0, t_unplug = 0, press_t = 0;
    uint16_t fx = 0, hue = 0, vf;
    uint8_t mode, color, prev_color = 0;
    uint8_t prev_touch = 1, armed = 0, fired = 1;
    uint8_t dirty = 0, crit_cnt = 0, critical = 0;
    uint8_t charging, full, was_charger = 0, settle = 0;
    uint8_t pct, pct_chg = 0;
    uint8_t candle = 180, candle_tgt = 180;
    uint8_t seg[4];

    /* touch: ulaz bez pull-upa; punjac: ulazi sa pull-upom */
    PD_DDR &= (uint8_t)~(TOUCH_BIT | CHRG_BIT | STDBY_BIT);
    PD_CR1 &= (uint8_t)~TOUCH_BIT;
    PD_CR1 |= (CHRG_BIT | STDBY_BIT);

    tim4_init();
    pwm_init();
    tm_init();

    color = EE_COLOR;
    if (color >= NCOLORS) color = 0;
    mode = EE_MODE;
    if (mode < 1 || mode > NMODES) mode = 1;

    wait_ms(20);
    vf  = vbat_mv();
    pct = pct_from_mv(vf);

    for (;;) {
        wait_ms(10);
        t++;
        fx++;

        /* ---------- punjac ---------- */
        charging = (PD_IDR & CHRG_BIT)  ? 0 : 1;
        full     = (PD_IDR & STDBY_BIT) ? 0 : 1;
        if (charging || full) {
            if (!was_charger) pct_chg = pct;     /* pocetak punjenja */
            was_charger = 1;
            if (critical) { critical = 0; crit_cnt = 0; }
            if (full) pct = 100;
        } else if (was_charger) {
            was_charger = 0;
            settle = 1;
            t_unplug = t;
        }

        /* ---------- touch ---------- */
        {
            uint8_t cur = (PD_IDR & TOUCH_BIT) ? 1 : 0;
            if (!armed) {
                if (t >= 60) armed = 1;          /* TTP223 se kalibrise ~0,5 s */
            } else if (!critical) {
                if (cur && !prev_touch) {        /* tek dodirnuto */
                    press_t = t;
                    fired = 0;
                    if (mode == 1 || mode == 3) {
                        prev_color = color;
                        color++;
                        if (color >= NCOLORS) color = 0;
                        dirty = 1; t_change = t;
                    }
                }
                if (cur && !fired && (uint16_t)(t - press_t) >= HOLD_TICKS) {
                    fired = 1;                   /* drzanje: sledeci mod */
                    if (mode == 1 || mode == 3) color = prev_color;   /* ponisti boju od dodira */
                    mode = (mode >= NMODES) ? 1 : (uint8_t)(mode + 1);
                    fx = 0;
                    dirty = 1; t_change = t;
                }
            }
            prev_touch = cur;
        }

        /* ---------- pamcenje (3 s posle poslednje promene) ---------- */
        if (dirty && (uint16_t)(t - t_change) >= 300) {
            ee_write(&EE_COLOR, color);
            ee_write(&EE_MODE, mode);
            dirty = 0;
        }

        /* ---------- baterija, jednom u sekundi ---------- */
        if (settle && (uint16_t)(t - t_unplug) >= 300) {
            vf  = vbat_mv();                     /* sveze merenje posle punjenja */
            pct = pct_from_mv(vf + (critical ? 0 : LOAD_COMP_MV));
            settle = 0;
            t_meas = t;
        }
        if (!settle && (uint16_t)(t - t_meas) >= 100) {
            uint16_t v = vbat_mv();
            t_meas = t;
            vf = (uint16_t)(((uint32_t)vf * 3 + v) / 4);

            if (was_charger) {
                uint8_t p = pct_from_mv(vf > CHG_COMP_MV ? vf - CHG_COMP_MV : 0);
                if (p > 99) p = 99;
                if (p > pct_chg) pct_chg = p;    /* dok se puni, samo raste */
            } else {
                uint8_t p = pct_from_mv(vf + (critical ? 0 : LOAD_COMP_MV));
                if (p < pct) pct = p;            /* dok radi, samo opada */

                if (vf < TH_CRIT) { if (crit_cnt < 255) crit_cnt++; }
                else crit_cnt = 0;
                if (crit_cnt >= 5) critical = 1;
            }
        }

        /* ---------- traka ---------- */
        if (critical) {
            R = G = B = 0;
        } else switch (mode) {
            case 1:                              /* stalna boja */
                R = PAL[color][0]; G = PAL[color][1]; B = PAL[color][2];
                break;
            case 2:                              /* duga */
                hue = (uint16_t)(hue + DUGA_KORAK);
                if (hue >= 1536) hue = (uint16_t)(hue - 1536);
                wheel(hue);
                break;
            case 3: {                            /* disanje u izabranoj boji */
                uint16_t k = fx % 400u;
                uint8_t lvl = (uint8_t)(k < 200u ? k : 400u - k);       /* 0..200  */
                uint8_t b8  = (uint8_t)(15u + (uint16_t)lvl * 240u / 200u);
                uint8_t g8  = (uint8_t)(((uint16_t)b8 * b8) >> 8);      /* gama   */
                R = (uint8_t)(((uint16_t)PAL[color][0] * g8) >> 8);
                G = (uint8_t)(((uint16_t)PAL[color][1] * g8) >> 8);
                B = (uint8_t)(((uint16_t)PAL[color][2] * g8) >> 8);
                break;
            }
            case 4: {                            /* sveca */
                if ((fx % 6u) == 0) {
                    uint8_t r = rnd();
                    candle_tgt = (uint8_t)(140u + (r & 0x73u));
                    if ((r & 0x0Fu) == 0) candle_tgt = 70;              /* povremeno "zatreperi" */
                }
                candle = (uint8_t)((int16_t)candle + ((int16_t)candle_tgt - (int16_t)candle) / 3);
                R = candle;
                G = (uint8_t)(((uint16_t)candle * 70u) >> 8);
                B = (uint8_t)(((uint16_t)candle * 6u) >> 8);
                break;
            }
            default: {                           /* 5: smena boja sa pretapanjem */
                uint16_t pos  = fx % SMENA_TICKS;
                uint8_t  i    = (uint8_t)((fx / SMENA_TICKS) % NCOLORS);
                uint8_t  j    = (uint8_t)((i + 1u) % NCOLORS);
                uint16_t fade = SMENA_TICKS / 3u;
                if (pos < SMENA_TICKS - fade) {
                    R = PAL[i][0]; G = PAL[i][1]; B = PAL[i][2];
                } else {
                    uint8_t f = (uint8_t)((pos - (SMENA_TICKS - fade)) * 255u / fade);
                    R = mix(PAL[i][0], PAL[j][0], f);
                    G = mix(PAL[i][1], PAL[j][1], f);
                    B = mix(PAL[i][2], PAL[j][2], f);
                }
                break;
            }
        }
        strip_rgb(R, G, B);

        /* ---------- displej, 10 puta u sekundi ---------- */
        if ((t % 10u) == 0) {
            uint8_t on = 1;
            uint8_t blink = (uint8_t)((t % 100u) < 50u);

            if (full) {
                seg[0] = SEG_F; seg[1] = SEG_U; seg[2] = SEG_L; seg[3] = SEG_L;
            } else if (charging) {
                seg[0] = DIG[mode];
                seg[1] = SEG_C;
                seg[2] = DIG[pct_chg / 10u];
                seg[3] = DIG[pct_chg % 10u];
            } else if (critical) {
                seg[0] = SEG_MINUS; seg[1] = SEG_L; seg[2] = SEG_O; seg[3] = SEG_MINUS;
                on = blink;
            } else {
                seg[0] = DIG[mode];
                seg[1] = (pct >= 100u) ? DIG[1] : SEG_BLANK;
                seg[2] = (pct >= 10u) ? DIG[(pct / 10u) % 10u] : SEG_BLANK;
                seg[3] = DIG[pct % 10u];
                if (vf < TH_LOW) on = blink;     /* slaba baterija: broj trepce */
            }
            tm_show(seg, on);
        }
    }
}
