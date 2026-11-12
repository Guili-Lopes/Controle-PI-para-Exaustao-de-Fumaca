/*
 * ============================================================================
 * PROJETO: Sistema de Controle PI de Exaustao de Fumaca
 * ============================================================================
 * Autores: Guilherme Lopes, Gabriel Araujo, Joao Manoel e Yan Pacheco
 * Disciplina: Fundamentos de Microcontroladores
 * Orientadora: Profa. Mara Cristina Coelho
 * Microcontrolador: PIC18F4550 
 *
 * OBJETIVO:
 * Desenvolver um sistema de exaustao com controle PI digital
 * implementado diretamente no PIC18F4550.
 * O potenciometro, define a referencia desejada; 
 * o sensor MQ-2, fornece a medida analogica.
 * O erro entre referencia e medida alimentara o PI.
 * A atuacao sera enviada ao CCP1, produzindo PWM para a ventoinha DC.
 * O tacografo permitira acompanhar a rotacao, e o LCD 16x2 mostrara
 * valores de referencia e leituras dos dados. Tambem sera implementado um 
 * sistema de sinalizacao de nivel alto de fumaca, com LED e buzzer.
 *
 * PARAMETROS PRELIMINARES:
 * FOSC = 20 MHz; PWM = 25 kHz; Timer0 = 10 ms; PI = 100 ms;
 * medida de RPM = 500 ms; atualizacao LCD = 250 ms.
 * 
 * PI: Kp = -1,5; Ki = -0,0675; Ts = 0,1 s; anti-windup = 0,5.
 *
 * PINOUT: 
 *  AN0 -> Potenciômetro para setpoint
 *  AN1 -> Sensor MQ-2
 *  RC0 -> Tacógrafo do FAN
 *  RC2 -> FAN
 *  RD0-RD7 -> LCD Dados
 *  RE0 -> LCD RS
 *  RE1 -> LCD EN
 *  RE2 -> LCD RW
 *  RB4 -> LED
 *  RB5 -> Buzzer
 *
*/


#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#define _XTAL_FREQ 20000000      // 20MHz necessário para as funções de delay



/*=========================
        PRAGMAS
===========================*/

// Oscilador principal HS a 20 MHz, sem PLL aplicado a CPU
#pragma config FOSC = HS
#pragma config PLLDIV = 5
#pragma config CPUDIV = OSC1_PLL2

// Recursos gerais
#pragma config WDT = OFF        // Watchdog desligado
#pragma config PBADEN = OFF     // PORTB inicia como digital
#pragma config LVP = OFF        // Gravacao em baixa tensao desabilitada
#pragma config DEBUG = OFF      // Programa normal;
#pragma config MCLRE = ON       // Mantem o reset externo
#pragma config PWRT = ON        // Temporizacao na energizacao



/*=========================
        DEFINES
===========================*/

// Tempos de operacao. Timer0 fornece a base de 10 ms
#define TICKS_PI 10u        // 10 x 10 ms = 100 ms
#define TICKS_LCD 25u       // 25 x 10 ms = 250 ms
#define TICKS_RPM 50u       // 50 x 10 ms = 500 ms

// Timer0: 16 bits, Fosc/4 = 5 MHz, prescaler 1:8
// 10 ms -> 6250 incrementos -> 65536 - 6250 = 59286
#define TIMER0_HIGH 0b11100111      // Parte alta
#define TIMER0_LOW 0b10010110       // Parte baixa

// PWM: 25 kHz -> periodo 40 us, prescaler 1:1, PR2 = 199
#define PWM_PR2 0b11000111      // 199 decimal
#define PWM_MAX 800u            // 4 x (PR2+1); limite de duty
#define ADC_MAX 1023u           // 10 bits

// Pinos/sinais da XM118. PORTA analogico nao deve ser lido como entrada digital
#define POT_IO TRISAbits.TRISA0             // AN0: entrada analogica (referencia)
#define SENSOR_IO TRISAbits.TRISA1          // AN1: entrada analogica (MQ-2)

// ADCON0: os bits CHS selecionam o canal; ADON=1 mantem ADC ligado.
#define ADC_POT_CONFIG 0b00000001           // CHS=0000 -> AN0; ADON=1
#define ADC_SENSOR_CONFIG 0b00000101        // CHS=0001 -> AN1; ADON=1
#define ADC_GO_MASK 0b00000010              // GO/DONE: inicia/conclui conversao
#define TACOGRAFO_IO TRISCbits.TRISC0       // RC0/T13CKI, entrada de pulsos
#define TACOGRAFO_PINO PORTCbits.RC0        // Apenas leitura digital
#define FAN_IO TRISCbits.TRISC2             // RC2/CCP1, saida PWM
#define FAN_LATCH LATCbits.LATC2           

// LCD alfanumerico 16x2, barramento de dados de 8 bits
#define LCD_DATA LATD               // D7..D0 em RD7..RD0
#define LCD_DATA_IO TRISD
#define LCD_RS LATEbits.LATE0       // RE0: 0=comando, 1=dado
#define LCD_EN LATEbits.LATE1       // RE1: pulso de habilitacao
#define LCD_RW LATEbits.LATE2       // RE2: 0=escrita, 1=leitura

// Sinalização de nível alto de fumaça com buzzer e LED
#define LED_IO TRISBbits.TRISB4         // RB4 como saída digital
#define LED_LATCH LATBbits.LATB4
#define BUZZER_IO TRISBbits.TRISB5     // RB5 como saída digital
#define BUZZER_LATCH LATBbits.LATB5
#define BUZZER_ON 0b1
#define BUZZER_OFF 0b0
#define LED_ON 0b0
#define LED_OFF 0b1

#define ALARM_VALUE 600u    // Valor de referência do sensor MQ-2 para acionar o alarme (LED e buzzer)



/*=========================
    PROTOTIPO DE FUNÇÕES
===========================*/

void config_regs(void);                             // Função responsável por configurar os registradores

void config_adc(void);                              // Função responsável por configurar o ADC
void read_adc(void);                                // Le AN0 (referencia) e AN1 (sensor MQ-2)
uint16_t read_adc_channel(uint8_t adc_config);      // Le um canal ADC de 10 bits

void config_pwm(void);                              // Função responsável por configurar o PWM 
void set_pwm_duty_cycle(void);                      // Função responsável por atualizar o duty cycle do PWM

void config_timers(void);                           // Função responsável por configurar os timers

void config_interrupts(void);                       // Função responsável por configurar as interrupções
void interrupt process_interrupts(void);            // ISR de Timer0

void calculate_pi(void);                            // Função responsável por realziar o controle PI
void calculate_rpm(void);                           // Função responsável por calcular o RPM da ventoinha

void config_lcd(void);                                      // Função responsável por configurar o LCD
void write_lcd(void);                                       // Escrever os dados no LCD
void lcd_write_text(const char *text);                      // Escreve uma string de caracteres no LCD
void lcd_write_number(uint16_t value, uint8_t digits);      // Escreve um numero decimal no LCD com quantidade fixa de digitos
void lcd_send_command(uint8_t command);                     // Envia instrucao ao LCD
void lcd_send_data(uint8_t data) ;                          // Envia dados ao LCD



/*=========================
    VARIAVÉIS GLOBAIS
===========================*/

const float Kp = -1.5f;
const float Ki = -0.0675f;
const float Ts = 0.1f;
const float anti_windup = 0.5f;   

uint16_t setpoint = 0;              // Potenciometro AN0: 0..1023
uint16_t sensor_value = 0;          // Sensor AN1: 0..1023
int16_t error = 0;                  // Erro bruto: -1023..+1023   
float error_norm = 0.0f;            // Erro normalizado: -1.0..+1.0
float proportional = 0.0f;          // Termo proporcional do PI
float integral = 0.0f;              // Termo integrador com anti-windup
float control_output = 0.0f;        // Saida do controlador PI

uint16_t pwm_duty_cycle = 0;        // Duty do CCP1: 0..800

uint16_t rpm = 0;                   // RPM calculado a partir do tacografo

volatile bool flag_pi = 0;          // Flag de controle do PI
volatile bool flag_lcd = 0;         // Flag de atualizacao do LCD
volatile bool flag_rpm = 0;         // Flag de calculo do RPM



/*=========================
        FUNÇÃO MAIN
===========================*/

void main(void) {
    config_regs();        // Configura os registradores
    config_adc();         // Configura o ADC
    config_pwm();         // Configura o PWM
    config_lcd();         // Inicializa o LCD
    config_timers();      // Configura e liga os timers
    config_interrupts();  // Habilita as interrupcoes

    while(1) {
        // A cada 100 ms: controlador PI e PWM
        if (flag_pi) {      // Verifica se a flag de controle do PI foi acionada
            flag_pi = false;        // Limpa a flag de controle do PI
            calculate_pi();         // Chama a funcao de calculo do PI
            set_pwm_duty_cycle();   // Atualiza o duty cycle do PWM
        }

        // A cada 500 ms: calculo do RPM
        if (flag_rpm) {     // Verifica se a flag de calculo do RPM foi acionada
            flag_rpm = false;       // Limpa a flag de calculo do RPM
            calculate_rpm();        // Chama a funcao de calculo do RPM
        }

        // A cada 250 ms: atualizacao do LCD
        if (flag_lcd) {     // Verifica se a flag de atualizacao do LCD foi acionada
            flag_lcd = false;       // Limpa a flag de atualizacao do LCD
            write_lcd();            // Chama a funcao de atualizacao do LCD
        }

        if(sensor_value >= ALARM_VALUE) {   // Verifica se o valor do sensor ultrapassou o limite de alarme
            LED_LATCH = LED_ON;         // Aciona o LED de alarme
            BUZZER_LATCH = BUZZER_ON;   // Aciona o buzzer de alarme
        } else {
            LED_LATCH = LED_OFF;        // Desliga o LED de alarme
            BUZZER_LATCH = BUZZER_OFF;  // Desliga o buzzer de alarme
        }

    }
}



/*=========================
        FUNÇÕES DE
    CONFIGuRAÇÃO DOS REGS
===========================*/

//--------------------------------------
// Inicializacao geral dos registradores
//--------------------------------------
void config_regs(void) {
    // Desabilita os comparadores analogicos
    CMCON = 0b00000111;

    // Inicializa LED e buzzer desligados
    // Antes de configurar os pinos como saidas
    LATBbits.LATB4 = LED_OFF;
    LATBbits.LATB5 = BUZZER_OFF;

    // TRIS: 1 = entrada, 0 = saida
    TRISA = 0b00111111;       // RA0 e RA1 como entradas analogicas
    TRISB = 0b11001111;       // RB4: LED; RB5: buzzer
    TRISC = 0b11111011;       // RC0: tacografo; RC2: PWM da ventoinha
    TRISD = 0b00000000;       // RD0-RD7: dados do LCD
    TRISE = 0b00000000;       // RE0: RS; RE1: EN; RE2: RW
}

//--------------------------------------
// Configuracao do conversor A/D
//--------------------------------------
void config_adc(void) {
    POT_IO = 0b1;     // RA0/AN0: entrada analogica do potenciometro
    SENSOR_IO = 0b1;  // RA1/AN1: entrada analogica do sensor MQ-2

    // ADCON1: x x VCFG1 VCFG0 PCFG3 PCFG2 PCFG1 PCFG0
    // 00: VREF+ = VDD (5 V), VREF- = VSS (0 V)
    // 1101: somente AN0 e AN1 analogicos; os demais canais digitais
    ADCON1 = 0b00001101;

    // ADCON2: 1 xxx 111 010
    // ADFM=1: resultado de 10 bits justificado a direita
    // ACQT=111: aquisicao automatica de 20 TAD = 32 us
    // ADCS=010: clock Fosc/32 -> TAD = 1,6 us
    ADCON2 = 0b10111010;

    // ADCON0: CHS=0000 -> canal AN0; ADON=1 -> ADC ligado
    // GO/DONE=0 -> nenhuma conversao iniciada ainda.
    // Apos habilitar o ADC, as demais inicializacoes garantem
    // mais de 2 us antes da primeira conversao.
    ADCON0 = ADC_POT_CONFIG;
}

//--------------------------------------
// Preparacao do modulo CCP1 para PWM
//--------------------------------------
void config_pwm(void) {
    FAN_IO = 0b0;       // RC2 como saida PWM
    FAN_LATCH   = 0b0;  // Saida inicialmente em nivel baixo

    // PR2 = 199: com Fosc=20 MHz e prescaler Timer2=1:1
    // Tpwm = (199+1) * 4 * 50ns = 40 us -> 25 kHz
    PR2 = PWM_PR2;  // 1100 0111

    // CCPR1L e os bits DC1B1:DC1B0 comecam zerados: duty = 0%
    CCPR1L = 0b00000000;

    // CCP1CON: 00 00 11 00 -> PWM de saida unica (CCP1 em RC2)
    // bits menos significativos do duty zerados
    CCP1CON = 0b00001100;
}

//--------------------------------------
// Configuracao de Timer0, Timer1 e Timer2
//--------------------------------------
void config_timers(void) {
    // TIMER1: contador externo de 16 bits em RC0/T13CKI
    // 0b10000010: RD16=1, clock externo=1, prescaler=1:1
    // T1OSCEN=0, T1SYNC=0 (sincronizado), TMR1ON=0
    TMR1H = 0b00000000;
    TMR1L = 0b00000000;
    T1CON = 0b10000011;  // Mesmo ajuste agora TMR1ON=1

    // TIMER2: base de tempo do PWM do CCP1
    TMR2  = 0b00000000;
    // 0b00000100: postscaler 1:1, TMR2ON=1, prescaler 1:1
    T2CON = 0b00000100;

    // TIMER0: 16 bits, clock Fosc/4, prescaler 1:8
    // T0CON = 0b10000010: TMR0ON=1, T08BIT=0 (16 bits)
    // T0CS=0 (clock interno), PSA=0 (prescaler habilitado)
    // T0PS=010 (1:8). O timer so sera ligado apos configurar LCD
    TMR0H = TIMER0_HIGH;
    TMR0L = TIMER0_LOW;
    T0CON = 0b10000010;
}

//--------------------------------------
// Inicializacao do LCD 16x2
//--------------------------------------
void config_lcd(void) {

    LCD_RS = 0b0;   // Comandos, nao caracteres
    LCD_RW = 0b0;   // Modo escrita
    LCD_EN = 0b0;   // Enable inativo

    __delay_ms(20);

    // Sequencia de sincronizacao para modo de dados de 8 bits
    lcd_send_command(0b00110000);
    __delay_ms(5);

    lcd_send_command(0b00110000);
    __delay_us(150);

    lcd_send_command(0b00110000);

    lcd_send_command(0b00111000);       // 8 birs, 2 linhas, caracteres 5x8
    lcd_send_command(0b00001000);       // Display desligado durante a configuracao
    lcd_send_command(0b00000001);       // Limpa display
    lcd_send_command(0b00000110);       // Incrementa cursor, sem deslocar display
    lcd_send_command(0b00001100);       // Display ligado, cursosr e piscagem desligados
}

//--------------------------------------
// Configuracao geral das interrupcoes
//--------------------------------------
void config_interrupts(void) {
    // IPEN=0: Sem prioridades de interrupcao
    RCONbits.IPEN = 0b0;

    // GIE=1: Habilita interrupcoes globais
    // PEIE=0: Interrupcoes de perifericos desabilitadas
    // TMR0IE=1: Habilita interrupcao do Timer0
    // TMR0IF=0: Limpa a flag do Timer0
    // Demais interrupcoes desabilitadas
    INTCON = 0b10100000;
}



/*=========================
    FUNÇÕES DE LEITURA
===========================*/

//--------------------------------------
// Leitura do valor de referência e do sensor
//--------------------------------------
void read_adc(void) {
    // Leitura do potenciometro (AN0)
    setpoint = read_adc_channel(ADC_POT_CONFIG);

    // Leitura do sensor MQ-2 (AN1)
    sensor_value = read_adc_channel(ADC_SENSOR_CONFIG);

    // Calcula o erro bruto entre as leituras
    error = setpoint - sensor_value;
    error = (error > 1023) ? 1023 : error;          // Limita o erro a 10 bits
    error = (error < -1023) ? -1023 : error;        // Limita o erro a 10 bits
    error_norm = (float)error / (float)ADC_MAX;     // Normaliza o erro para -1.0 a +1.0
}

//--------------------------------------
// Leitura do canal ADC de 10 bits
//--------------------------------------
uint16_t read_adc_channel(uint8_t adc_config) {
    ADCON0 = adc_config;
    ADCON0bits.GO = 0b1;
    while (ADCON0bits.GO);  // Aguarda concluir conversao

    return (((uint16_t)ADRESH << 8) | ADRESL); // Resultado de 10 bits
}

/*==========================
  FUNÇÕES DE PROCESSAMENTO
===========================*/

//--------------------------------------
// Tratando interrupções
//--------------------------------------
void interrupt process_interrupts(void) {

    // Contadores das interrupcoes do Timer0
    static uint8_t ticks_pi = 0;
    static uint8_t ticks_lcd = 0;
    static uint8_t ticks_rpm = 0;

    // Verifica se a interrupcao foi causada pelo Timer0
    if (INTCONbits.TMR0IF == 0b1) {

        // Recarrega o Timer0 para aproximadamente 10 ms
        TMR0H = TIMER0_HIGH;
        TMR0L = TIMER0_LOW;

        // Incrementa os contadores de tempo
        ticks_pi++;
        ticks_lcd++;
        ticks_rpm++;

        // A cada 100 ms: controlador PI
        if (ticks_pi >= TICKS_PI) {
            ticks_pi = 0;
            flag_pi = true;
        }

        // A cada 250 ms: atualizacao do LCD
        if (ticks_lcd >= TICKS_LCD) {
            ticks_lcd = 0;
            flag_lcd = true;
        }

        // A cada 500 ms: calculo do RPM
        if (ticks_rpm >= TICKS_RPM) {
            ticks_rpm = 0;
            flag_rpm = true;
        }
    }

    // Limpa a flag de interrupcao do Timer0
    INTCONbits.TMR0IF = 0b0;
}

//--------------------------------------
// Calculando o PI
//--------------------------------------
void calculate_pi(void) {
    // Leituras do sensor e do potenciômetro 
    read_adc();

    // Parcelas PI
    integral += Ki * Ts * error_norm;
    proportional = Kp * error_norm;

    // Limitador do integrador (anti-windup) 
    // Clamping do integrador para evitar saturação
    if (integral > anti_windup) integral = anti_windup;
    if (integral < -anti_windup) integral = -anti_windup;

    // Sinal de Saída
    // PWM base de 50% + correcao do PI
    control_output = 0.5f + integral + proportional;

    // Saturacao do PWM entre 0% e 100%
    if (control_output > 1.0f) control_output = 1.0f;
    if (control_output < 0.0f) control_output = 0.0f;

    pwm_duty_cycle = (uint16_t)(control_output * PWM_MAX); // Conversão para duty cycle do PWM
}   

//--------------------------------------
// Calculando o RPM a partir do tacografo
//--------------------------------------
void calculate_rpm(void) {
    static uint16_t previous_count = 0;     // Medição anterior do contador do Timer1
    uint8_t timer1_low;
    uint8_t timer1_high;
    uint16_t current_count;
    uint16_t pulse_count;

    timer1_low = TMR1L;
    timer1_high = TMR1H;

    current_count = ((uint16_t)timer1_high << 8) | timer1_low;  // Juntando os dois regs em uma variavel de 16 bits
    
    pulse_count = (uint16_t)(current_count - previous_count);   // Calcula os pulsos desde a ultima leitura
    previous_count = current_count;                             // Armazena a contagem atual para a próxima leitura

    // Calcula o RPM
    rpm = (pulse_count * 30);   
}

//--------------------------------------
// Atualizando o duty cycle do PWM
//--------------------------------------
void set_pwm_duty_cycle(void) {
    // Limita o duty cycle ao valor maximo permitido
    if (pwm_duty_cycle > PWM_MAX) pwm_duty_cycle = PWM_MAX;

    // Armazena os 8 bits mais significativos do duty
    CCPR1L = (uint8_t)(pwm_duty_cycle >> 2);

    // Armazena os 2 bits menos significativos
    // nos bits 5 e 4 do CCP1CON
    // Preserva os demais bits do registrador
    CCP1CON = (CCP1CON & 0b11001111) | ((pwm_duty_cycle & 0b00000011) << 4);
}

//--------------------------------------
// Envio de comando (RS=0) ao LCD
//--------------------------------------

void lcd_send_command(uint8_t command) {

    LCD_RS = 0b0;       // Modo comando
    LCD_RW = 0b0;       // Modo escrita

    LCD_DATA = command; // Envia comando ao LATD

    __delay_us(1);      // Estabilizacao dos sinais

    LCD_EN = 0b1;       // Inicia pulso de habilitacao
    __delay_us(1);
    LCD_EN = 0b0;       // Finaliza pulso

    // Comandos de limpar e retornar o cursor
    if (command == 0b00000001 || command == 0b00000010) {
        __delay_ms(2);
    }
    else {
        __delay_us(50);
    }
}

//--------------------------------------
// Envio de dado ao LCD
//--------------------------------------
void lcd_send_data(uint8_t data) {

    LCD_RS = 0b1;       // RS = 1: envio de caractere
    LCD_RW = 0b0;       // RW = 0: escrita no LCD

    LCD_DATA = data;    // Escreve os 8 bits no LATD

    __delay_us(1);     // Estabilizacao dos dados(Delay de 1ms)

    LCD_EN = 0b1;       // Inicia pulso de habilitacao
    __delay_us(1);     // Pulso de habilitacao (Delay de 1ms)
    LCD_EN = 0b0;       // Finaliza pulso

    __delay_us(50);   // Aguarda o processamento do LCD
}


//--------------------------------------
// Escreve uma string de caracteres no LCD
//--------------------------------------
void lcd_write_text(const char *text) {

    while (*text != '\0') {
        lcd_send_data((uint8_t)*text);
        text++;
    }
}

//--------------------------------------
//      Escreve um numero decimal 
//  no LCD com quantidade fixa de digitos
//--------------------------------------
void lcd_write_number(uint16_t value, uint8_t digits) {

    uint16_t divisor = 1;

    for (uint8_t i = 1; i < digits; i++) {
        divisor *= 10;
    }

    while (divisor > 0) {
        lcd_send_data((uint8_t)('0' + (value / divisor) % 10));

        divisor /= 10;
    }
}

//--------------------------------------
// Escreve os dados no LCD
//--------------------------------------

void write_lcd(void) {

    uint8_t pwm_percent;

    // Converte o duty cycle para porcentagem
    pwm_percent = (uint8_t)(pwm_duty_cycle / 8u);

    // PRIMEIRA LINHA: referencia e sensor MQ-2
    lcd_send_command(0b10000000);

    lcd_write_text("REF:");
    lcd_write_number(setpoint, 4);

    lcd_write_text(" MQ:");
    lcd_write_number(sensor_value, 4);

    // SEGUNDA LINHA: PWM e RPM
    lcd_send_command(0b11000000);

    lcd_write_text("PWM:");
    lcd_write_number(pwm_percent, 3);

    lcd_write_text("% R:");
    lcd_write_number(rpm, 5);
}
