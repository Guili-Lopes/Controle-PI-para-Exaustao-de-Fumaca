# Sistema de Controle PI para Exaustão de Fumaça

**Projeto acadêmico de Fundamentos de Microcontroladores — PIC18F4550 | EXSTO XM118**

Este projeto desenvolve um sistema embarcado de exaustão de fumaça com controle de velocidade de uma ventoinha DC. O **PIC18F4550** realiza a leitura de um potenciômetro e de um sensor **MQ-2** pelo conversor A/D, calcula a atuação de um controlador PI digital e ajusta a ventoinha por meio de **PWM gerado pelo módulo CCP1**. O sistema também mede a rotação por pulsos externos, apresenta informações em um **LCD 16×2** e aciona um **LED e um buzzer** quando a leitura do sensor ultrapassa um limite estabelecido.

O foco do trabalho é a **configuração e integração dos periféricos do microcontrolador**: GPIO, ADC, temporizadores, interrupções, contador externo, PWM e interface paralela com LCD. O repositório reúne o firmware em C, o esquemático no KiCad e as simulações em MATLAB, Simulink e PICSimLab.

## 1. Funcionamento do sistema

1. Um **potenciômetro em AN0** estabelece a referência desejada.
2. O **MQ-2 em AN1** fornece um sinal analógico relacionado às condições detectadas pelo sensor.
3. O **ADC de 10 bits** converte ambos os sinais em valores de 0 a 1023.
4. A cada **100 ms**, o PIC executa o controle PI e atualiza o duty cycle do **PWM em RC2/CCP1**.
5. Um sensor óptico fornece pulsos ao **Timer1 em RC0**, permitindo estimar a rotação da ventoinha a cada **500 ms**.
6. O **LCD 16×2** apresenta referência, leitura do sensor, duty cycle e RPM, com atualização a cada **250 ms**.
7. Quando a leitura do MQ-2 é **igual ou superior a 600**, o LED e o buzzer externos são acionados continuamente; abaixo desse valor, são desligados.

## 2. Principais configurações do PIC18F4550

| Recurso | Configuração adotada | Finalidade |
|---|---|---|
| Oscilador | `FOSC = 20 MHz`, modo `HS` | Clock principal do microcontrolador |
| Clock de instruções | `FOSC/4 = 5 MHz` | Referência para cálculos dos timers |
| ADC | 10 bits, canais AN0 e AN1 | Leitura da referência e do MQ-2 |
| Timer0 | 16 bits, clock interno, prescaler 1:8 | Interrupção de base a cada ~10 ms |
| Timer1 | 16 bits, clock externo em RC0, prescaler 1:1 | Contagem dos pulsos do tacômetro |
| Timer2 | Prescaler 1:1, `PR2 = 199` | Base de tempo do PWM |
| CCP1 | Modo PWM em RC2, 25 kHz | Acionamento da ventoinha |
| LCD | 16×2, interface paralela de 8 bits | Exibição dos valores do sistema |
| RB4 / RB5 | Saídas digitais | LED e buzzer de sinalização |

### 2.1. Oscilador de 20 MHz

O firmware foi desenvolvido para um **clock de 20 MHz**, configurado por meio de `#pragma config FOSC = HS`. A constante abaixo informa essa frequência às funções de atraso do compilador XC8:

```c
#define _XTAL_FREQ 20000000
```

Nos PIC18, o clock de instruções é obtido dividindo a frequência do oscilador por quatro:

$$
F_{cy}=\frac{F_{OSC}}{4}=\frac{20\,000\,000}{4}=5\,000\,000\ \text{Hz}
$$

Portanto, cada ciclo de instrução corresponde a **0,2 µs**. Esse valor é a base para dimensionar os temporizadores e o PWM. A configuração `HS` foi adotada para operação com o cristal externo de 20 MHz previsto no projeto.

### 2.2. Timer0 — base de tempo de 10 ms

O **Timer0** opera em **16 bits**, usando o clock interno `FOSC/4` e prescaler **1:8**:

```c
T0CON = 0b10000010;
```

O intervalo entre incrementos é:

$$
T_{tick}=\frac{4}{F_{OSC}}\times8=1{,}6\ \mu s
$$

Para gerar um intervalo de **10 ms**, precisamos de:

$$
N=\frac{10\ ms}{1{,}6\ \mu s}=6250\ \text{incrementos}
$$

Como o contador é de 16 bits, ele transborda em 65536 contagens. O valor de recarga escolhido foi:

$$
TMR0=65536-6250=59286=\texttt{0xE796}
$$

No firmware:

```c
#define TIMER0_HIGH 0b11100111  // 0xE7
#define TIMER0_LOW  0b10010110  // 0x96
```

A interrupção do Timer0 atualiza contadores de software e sinaliza tarefas na `main()` por meio de flags `volatile`:

| Tarefa | Interrupções de 10 ms | Intervalo nominal |
|---|---:|---:|
| Executar PI e atualizar PWM | 10 | 100 ms |
| Atualizar LCD | 25 | 250 ms |
| Calcular RPM | 50 | 500 ms |

O processamento das tarefas ocorre **fora da rotina de interrupção**, mantendo a ISR curta. Os períodos são aproximados, pois o tempo de atendimento e recarga do timer também influencia a temporização real.

### 2.3. Timer2 e CCP1 — PWM de 25 kHz

O **Timer2** fornece a base de tempo do módulo **CCP1**, responsável por gerar o PWM na saída **RC2**. Foram escolhidos:

- **Frequência do oscilador:** 20 MHz;
- **Prescaler do Timer2:** 1:1;
- **Registrador `PR2`:** 199 (`0b11000111`);
- **Frequência resultante do PWM:** 25 kHz.

Para o PWM do CCP1:

$$
F_{PWM}=\frac{F_{OSC}}{4\,(PR2+1)\,P_{TMR2}}
$$

Substituindo os valores:

$$
F_{PWM}=\frac{20\,000\,000}{4\times(199+1)\times1}=25\,000\ \text{Hz}
$$

Assim, o período do PWM é **40 µs**. A frequência de **25 kHz** foi adotada para permitir o controle de velocidade com chaveamento acima da faixa audível usual, reduzindo a possibilidade de ruído tonal do acionamento, sem garantir silêncio absoluto do conjunto eletromecânico.

O valor de duty cycle utilizado pelo CCP1 é limitado a:

$$
PWM_{MAX}=4(PR2+1)=4\times200=800
$$

Por isso, o firmware trabalha com valores de **0 a 800**, correspondentes a **0% a 100%** de duty cycle. Os oito bits superiores são escritos em `CCPR1L` e os dois bits inferiores nos campos `DC1B1:DC1B0` de `CCP1CON`.

```c
PR2 = 0b11000111;
T2CON = 0b00000100;   // Timer2 ligado; prescaler 1:1
CCP1CON = 0b00001100; // CCP1 em modo PWM
```

### 2.4. Timer1 — medição de rotação

O **Timer1** é configurado como um **contador externo de 16 bits**, recebendo pulsos no pino **RC0/T13CKI**:

```c
T1CON = 0b10000011;
```

O firmware lê o contador a cada **500 ms** e calcula a diferença em relação à leitura anterior. Considerando, como hipótese de projeto, **quatro pulsos por revolução**:

$$
RPM=\frac{\text{pulsos}\times60}{0{,}5\times4}=\text{pulsos}\times30
$$

Essa relação depende do sensor óptico realmente produzir quatro pulsos por volta. O número efetivo de pulsos deve ser confirmado durante os testes do tacômetro, e o intervalo de amostragem deve permanecer próximo de 500 ms.

### 2.5. ADC — leitura do potenciômetro e do MQ-2

O conversor A/D utiliza **resolução de 10 bits**, com referências em **VDD/VSS** e apenas os canais **AN0 e AN1** configurados como analógicos:

```c
ADCON1 = 0b00001101;
ADCON2 = 0b10111010;
```

A configuração de `ADCON2` seleciona resultado justificado à direita, tempo de aquisição automático de **20 TAD** e clock A/D `FOSC/32`. Com `FOSC = 20 MHz`, temos `TAD = 1,6 µs` e aquisição de **32 µs**.

- **AN0:** leitura do potenciômetro, utilizada como referência do controlador.
- **AN1:** leitura analógica do sensor MQ-2.

Os valores de ambos são apresentados no LCD em **contagens do ADC (0–1023)**, evitando interpretá-los incorretamente como porcentagem real de fumaça.

### 2.6. LCD 16×2 — interface de 8 bits

A comunicação com o LCD é feita diretamente pelos registradores do PIC:

- **RD0–RD7:** barramento paralelo de dados;
- **RE0:** seleção entre comando e caractere (`RS`);
- **RE1:** habilitação (`EN`);
- **RE2:** direção da operação (`RW`), mantida em escrita.

Exemplo do conteúdo apresentado:

```text
REF:0307 MQ:0614
PWM:080% R:03000
```

A inicialização inclui a sequência de configuração do controlador compatível com **HD44780**. Durante a escrita, são utilizadas pequenas esperas com `__delay_us()`, distinguindo instruções comuns (50 µs) dos comandos `Clear Display` e `Return Home` (2 ms). O display é atualizado sem executar uma limpeza completa a cada ciclo, reduzindo cintilação e tempo de bloqueio.

### 2.7. Entradas, saídas e sinalização

| Pino | Uso |
|---|---|
| RA0 / AN0 | Potenciômetro de referência |
| RA1 / AN1 | Saída analógica do MQ-2 |
| RC0 / T13CKI | Entrada de pulsos do tacômetro |
| RC2 / CCP1 | PWM da ventoinha |
| RD0–RD7 | Dados do LCD |
| RE0 / RE1 / RE2 | RS / EN / RW do LCD |
| RB4 | LED externo de alarme |
| RB5 | Buzzer externo de alarme |
| RB0 / RB1 | Direção do motor virtual, **somente na configuração do PICSimLab** |

No esquemático físico, a etapa de potência utiliza um **ULN2803A** para o acionamento de cargas, e o projeto inclui a conversão de alimentação de **12 V para 5 V**. A lógica do LED é ativa em nível baixo; a do buzzer externo, em nível alto.

O alarme é acionado quando a leitura do MQ-2 alcança **600 contagens ADC**. Trata-se de um limiar **experimental**, que não corresponde a um valor calibrado de concentração de gás ou fumaça.

## 3. Controle PI digital (resumo)

O controle PI foi utilizado para relacionar a referência do potenciômetro e a leitura do sensor à atuação sobre a ventoinha. O firmware adota **`Kp = -1,5`**, **`Ki = -0,0675`** e período de amostragem de **`Ts = 0,1 s`**. A integral é atualizada por **Euler regressivo**, limitada à faixa **−0,5 a +0,5**; a saída é combinada com uma atuação-base de **50%** e saturada entre **0% e 100%** antes de atualizar o CCP1.

Os estudos em **MATLAB** e **Simulink** apresentam comparações do comportamento do PI, incluindo efeitos de saturação e estratégias de limitação da integral. O objetivo central da implementação embarcada, entretanto, é demonstrar a integração entre amostragem, processamento e periféricos do PIC18F4550.

## 4. Estrutura do repositório

```text
Controle-PI-para-Exaustao-de-Fumaca/
├── Firmware
│   └── main.c
├── Esquemático_KiCad/
│   ├── Micro.kicad_pro
│   ├── Micro.kicad_sch
│   ├── Esquemático.pdf
│   └── ... (folhas hierárquicas do circuito)
├── Placa PICSimLab/
│   └── Board.pzw
├── Resultados_Simulacao_Matlab/
│   ├── Simulacao_PI_MATLAB.m
│   ├── ComparacaoPI.png
│   └── ... (gráficos, dados e métricas)
└── Resultados_Simulacao_Simulink/
    ├── comparacao_PI_Simulink.slx
    ├── Criar_Modelo_PI_Simulink.m
    └── ... (scripts, gráficos e métricas)
```

**Arquivos e materiais:**

- [Firmware `main.c`](Firmware/main.c)
- [Esquemático completo em PDF](Esquem%C3%A1tico_KiCad/Esquem%C3%A1tico.pdf)
- [Projeto KiCad](Esquem%C3%A1tico_KiCad/)
- [Placa configurada no PICSimLab](Placa%20PICSimLab/Board.pzw)
- [Simulações e resultados no MATLAB](Resultados_Simulacao_Matlab/)
- [Simulações e resultados no Simulink](Resultados_Simulacao_Simulink/)

## 5. Compilação e simulação

O programa foi escrito em **C para o compilador MPLAB XC8**, utilizando acesso direto aos registradores do **PIC18F4550**.

1. Crie um projeto para o **PIC18F4550** no **MPLAB X IDE** e selecione o compilador **XC8**.
2. Adicione o arquivo [`main.c`](main.c) ao projeto e configure o hardware para operação com **oscilador de 20 MHz**, conforme o esquemático.
3. Compile o projeto para gerar o arquivo **`.hex`**.
4. Para utilizar a **XM118 física**, mantenha comentadas as instruções exclusivas do motor virtual e preserve `TRISB = 0b11001111`.
5. Para utilizar o **PICSimLab**, abra a placa `Board.pzw`, habilite no código as definições de `MOTOR_IN1` e `MOTOR_IN2` e a inicialização da direção; utilize **`TRISB = 0b11001100` no lugar da configuração da XM118**. Recompile e carregue o `.hex` correspondente no simulador.

Assim, **um mesmo código-fonte** atende aos dois ambientes, com pequenas mudanças de configuração nos pinos de direção do motor virtual. Para a simulação, a entrada analógica do MQ-2 pode ser representada por uma fonte analógica variável, conforme a montagem da placa no PICSimLab.

## 6. Equipe

**Disciplina:** Fundamentos de Microcontroladores  
**Orientadora:** Profa. Mara Cristina Coelho

**Integrantes:**

- Guilherme Lopes
- Gabriel Araujo
- Joao Manoel
- Yan Pacheco

---

**Projeto acadêmico desenvolvido para estudo e aplicação prática dos periféricos do PIC18F4550.**
