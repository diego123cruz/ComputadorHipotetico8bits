// ============================================================
// COMPUTADOR HIPOTÉTICO 8 BITS - 09/2026
// CPU v2.1
//
// Arduino Nano / ATmega328P
//
// RAM: 256 bytes
//
// Registradores:
//   A  = 8 bits
//   B  = 8 bits
//   PC = 8 bits
//
// FLAGS:
//   N = Negative
//   Z = Zero
//   C = Carry
//   V = Overflow
//
// CONTROL:
//   b0 = MEM
//   b1 = RUN
//   b2 = I/O
//   b3 = HALT
//   b4 = V
//   b5 = C
//   b6 = Z
//   b7 = N
//
// ============================================================
#include <EEPROM.h>
#include <Wire.h>

// ============================================================
// HARDWARE
// ============================================================

// ------------------------------------------------------------
// 74HC165 - Entradas
// ------------------------------------------------------------

#define latchPin 5
#define clockPin 6
#define dataPin  7


// ------------------------------------------------------------
// 74HC595 - Saídas
// ------------------------------------------------------------

#define DATA_PIN  2
#define LATCH_PIN 3
#define CLOCK_PIN 4

// ------------------------------------------------------------
// Botões diretos
// ------------------------------------------------------------

#define inStep A0
#define inAddr A1
#define inRun  A2
#define inClr  A3
#define inIncr 9

// led saida (OUTA)
#define ledOutAddr 0x20
//#define ledOutI2C A4
//#define ledOutI2C A5


// ============================================================
// CONFIGURAÇÃO DOS BOTÕES
// ============================================================

const byte pinosEntrada[] = {
  inStep,
  inAddr,
  inRun,
  inClr,
  inIncr
};

const char* nomesPinos[] = {
  "Step",
  "Addr",
  "Run",
  "Clr",
  "Incr"
};

const byte totalPinos = 5;

byte estadosAnteriores[totalPinos];


// ============================================================
// ENTRADA 8 BITS
// ============================================================

byte dadosBotoes = 0;
byte estadoAnteriorBotoes = 0xFF;


// ============================================================
// BARRAMENTO / PAINEL
// ============================================================

byte addr = 0;
byte data = 0;
byte controle = 0;


// ============================================================
// MEMÓRIA
// ============================================================

byte memory[256];


// ============================================================
// REGISTRADORES DA CPU
// ============================================================

byte A = 0;
byte B = 0;
byte PC = 0;


// ============================================================
// FLAGS
// ============================================================
//
// bit 0 = C
// bit 1 = Z
// bit 2 = N
// bit 3 = V
//
// ============================================================

#define FLAG_C 0
#define FLAG_Z 1
#define FLAG_N 2
#define FLAG_V 3

#define FLAG_C_MASK (1 << FLAG_C)
#define FLAG_Z_MASK (1 << FLAG_Z)
#define FLAG_N_MASK (1 << FLAG_N)
#define FLAG_V_MASK (1 << FLAG_V)

byte flags = 0;


// ============================================================
// BITS DO BYTE CONTROLE
// ============================================================

#define CTRL_MEM  0
#define CTRL_RUN  1
#define CTRL_IO   2
#define CTRL_HALT 3
#define CTRL_V    4
#define CTRL_C    5
#define CTRL_Z    6
#define CTRL_N    7


// ============================================================
// ESTADO DA CPU
// ============================================================

enum CPUState {
  CPU_STOPPED,
  CPU_RUNNING,
  CPU_WAIT_INPUT,
  CPU_HALTED,
  CPU_DELAY
};

CPUState cpuState = CPU_STOPPED;


// ============================================================
// ESTADOS AUXILIARES
// ============================================================

bool ioWaiting = false;
bool memMode = false;

// Indica que o DELAY atual foi iniciado por um STEP
bool delayPorStep = false;

// Indica que uma instrução está sendo executada através de STEP
bool executandoStep = false;

bool ioPorStep = false;



// ============================================================
// DELAY NÃO BLOQUEANTE
// ============================================================

unsigned long delayInicio = 0;
unsigned long delayDuracao = 0;


// ============================================================
// PULSO DO LED RUN PARA STEP
// ============================================================

bool stepPulse = false;
unsigned long stepPulseInicio = 0;

#define STEP_PULSE_TIME 80


// ============================================================
// OPCODES
// ============================================================

// ------------------------------------------------------------
// SYSTEM
// ------------------------------------------------------------
#define OP_NOP       0x00
#define OP_HALT      0x01

// ------------------------------------------------------------
// LDA
// ------------------------------------------------------------
#define OP_LDA_MEM   0x10
#define OP_LDA_IMM   0x11
#define OP_LDB_MEM   0x12
#define OP_LDB_IMM   0x13
#define OP_LDA_INDB  0x14   // LDA [B]  = A <- MEM[B]

// ------------------------------------------------------------
// STA
// ------------------------------------------------------------
#define OP_STA_MEM   0x20
#define OP_STA_INDB  0x21   // STA [B]  = MEM[B] <- A
#define OP_STB_MEM   0x22   // STB [addr] = MEM[addr] <- B

// ------------------------------------------------------------
// MOV
// ------------------------------------------------------------
#define OP_MOV_A_B   0x30
#define OP_MOV_B_A   0x31
#define OP_SWAP      0x32

// ------------------------------------------------------------
// ADD
// ------------------------------------------------------------
#define OP_ADD_B     0x40
#define OP_ADD_MEM   0x41
#define OP_ADD_IMM   0x42

// ------------------------------------------------------------
// SUB
// ------------------------------------------------------------
#define OP_SUB_B     0x50
#define OP_SUB_MEM   0x51
#define OP_SUB_IMM   0x52

// ------------------------------------------------------------
// LOGIC
// ------------------------------------------------------------
#define OP_OR_B      0x60
#define OP_OR_MEM    0x61

#define OP_AND_B     0x62
#define OP_AND_MEM   0x63

#define OP_XOR_B     0x64
#define OP_XOR_MEM   0x65
#define OP_XOR_IMM   0x66

#define OP_NOT_A     0x67
#define OP_NOT_B     0x68

#define OP_AND_IMM   0x69
#define OP_OR_IMM    0x6A

// ------------------------------------------------------------
// COMPARE
// ------------------------------------------------------------
#define OP_CP_B      0x70
#define OP_CP_MEM    0x71
#define OP_CP_IMM    0x72

// ------------------------------------------------------------
// INC / DEC
// ------------------------------------------------------------
#define OP_INC_A     0x80
#define OP_DEC_A     0x81

#define OP_INC_B     0x82
#define OP_DEC_B     0x83

#define OP_INC_MEM   0x84   // INC [addr] = MEM[addr] <- MEM[addr] + 1
#define OP_DEC_MEM   0x85   // DEC [addr] = MEM[addr] <- MEM[addr] - 1

// ------------------------------------------------------------
// JUMP
// ------------------------------------------------------------
#define OP_JP        0x90
#define OP_JPZ       0x91
#define OP_JPNZ      0x92
#define OP_JPC       0x93
#define OP_JPNC      0x94
#define OP_JPN       0x95
#define OP_JNN       0x96
#define OP_JPV       0x97
#define OP_JPNV      0x98

// ------------------------------------------------------------
// SUBROTINAS
// ------------------------------------------------------------
#define OP_CALL      0xD0   // CALL [addr] = MEM[RET_ADDR] <- PC ; PC <- addr
#define OP_RET       0xD1   // RET = PC <- MEM[RET_ADDR]
#define RET_ADDR     0xFF   // endereço reservado para o retorno de CALL/RET

// ------------------------------------------------------------
// SHIFT / ROTATE
// ------------------------------------------------------------
#define OP_SHL_A     0xA0
#define OP_SHR_A     0xA1
#define OP_ROL_A     0xA2
#define OP_ROR_A     0xA3

#define OP_SHL_B     0xA4
#define OP_SHR_B     0xA5
#define OP_ROL_B     0xA6
#define OP_ROR_B     0xA7


// ------------------------------------------------------------
// I/O
// ------------------------------------------------------------
#define OP_INA       0xB0
#define OP_OUTA      0xB1
#define OP_KEY       0xB2

// ------------------------------------------------------------
// MISC
// ------------------------------------------------------------
#define OP_RNDA      0xC0
#define OP_DLAYA     0xC1
#define OP_DLAYB     0xC2










// ============================================================
// PROTÓTIPOS
// ============================================================

void leEntrada8bits();
void leEntradasControles();

void atualizar595();
void atualizarControle();
void atualizarDataCPU();

void cpuStep();
byte fetch();
byte fetchOperand();

void executarOpcode(byte opcode);

void atualizaZN(byte valor);
void limpaCV();
bool flagAtiva(byte flag);

void aluADD(byte valor);
void aluSUB(byte valor);
void aluCMP(byte valor);
void aluINC(bool registroA);
void aluDEC(bool registroA);

void shiftLeft(bool registroA);
void shiftRight(bool registroA);
void rotateLeft(bool registroA);
void rotateRight(bool registroA);

void iniciarINA();
void executarOUTA();
void executarKEY();

void processarDelay();

void resetCPU();

void debugCPU();
void debugMemory();


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);


  // ----------------------------------------------------------
  // Entradas diretas
  // ----------------------------------------------------------

  for (byte i = 0; i < totalPinos; i++) {

    pinMode(pinosEntrada[i], INPUT);

    estadosAnteriores[i] =
      digitalRead(pinosEntrada[i]);
  }


  // ----------------------------------------------------------
  // 74HC165
  // ----------------------------------------------------------

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);

  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, HIGH);


  // ----------------------------------------------------------
  // 74HC595
  // ----------------------------------------------------------

  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  // ----------------------------------------------------------
  // Estado inicial
  // ----------------------------------------------------------

  resetCPU();

  atualizar595();


  Serial.println();
  Serial.println("==============================");
  Serial.println(" CPU 8 BITS v2.1");
  Serial.println("==============================");
  Serial.println("CPU pronta.");
}


// ============================================================
// LOOP PRINCIPAL
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // Entrada dos 8 bits
  // ----------------------------------------------------------

  leEntrada8bits();


  // ----------------------------------------------------------
  // Botões
  // ----------------------------------------------------------

  leEntradasControles();


  // ----------------------------------------------------------
  // Processa delay
  // ----------------------------------------------------------

  if (cpuState == CPU_DELAY) {

    processarDelay();
  }


  // ----------------------------------------------------------
  // CPU em RUN
  // ----------------------------------------------------------

  if (cpuState == CPU_RUNNING) {

    cpuStep();
  }


  // ----------------------------------------------------------
  // Pulso do LED RUN durante STEP
  // ----------------------------------------------------------

  if (stepPulse) {

    if (millis() - stepPulseInicio >= STEP_PULSE_TIME) {

      stepPulse = false;
    }
  }


  // ----------------------------------------------------------
  // Atualiza painel
  // ----------------------------------------------------------

  atualizar595();


  delay(1);
}

void ledOutSend(byte x) {
  Wire.beginTransmission(ledOutAddr);
  Wire.write(~x);
  Wire.endTransmission();
}

void loadSaveMemory() {
  // leds
  // memMode
  // ioWaiting
  memMode = true;
  atualizarControle();
  
  for(byte i=0; i <5; i++) {
    addr = 0x81;
    atualizar595();
    delay(100);
    addr = 0x00;
    atualizar595();
    delay(100);
  }

  bool isFinish = false;
  bool isLoading = true;
  byte slotMem = 0;
  while(!isFinish) {
    if(isLoading) {
      addr = 0x01;
    } else {
      addr = 0x80;
    }

    if(digitalRead(inStep) == LOW) {
      if(isLoading) {
        isLoading = false;
      } else {
        isLoading = true;
      }
      delay(200);
    }
    data = 0;
    leEntrada8bits();
    data = data & 0x0F;
    if (data > 0) {
      slotMem = data;
    }

    if(slotMem > 0) {
      ioWaiting = true;
      data = slotMem;
    
      if(digitalRead(inIncr) == LOW) {
        int eepromOffset = 0;
        
        if(data == 1) eepromOffset = 0 * 256; // Slot 0
        if(data == 2) eepromOffset = 1 * 256; // Slot 1
        if(data == 4) eepromOffset = 2 * 256; // Slot 2
        if(data == 8) eepromOffset = 3 * 256; // Slot 3
        
        for(int j=0; j<256; j++) {
          addr = j;
          if (isLoading) {
            int address = eepromOffset+j;
            byte value = EEPROM.read(address);
            memory[j] = value;
            data = value;
          } else {
            // save
            int address = eepromOffset+j;
            byte value = memory[j];
            EEPROM.update(address, value);
            data = value;
          }
          atualizar595();
          delay(5);
        }
        
        isFinish=true;
      }
    } else {
      ioWaiting = false;
    }

    atualizar595();
    delay(1);
    if (digitalRead(inClr) == LOW) {
      isFinish=true;
    }
  }
  delay(100);
}


// ============================================================
// CPU STEP
// ============================================================

void cpuStep() {

  // ----------------------------------------------------------
  // Estados que impedem execução
  // ----------------------------------------------------------

  if (cpuState == CPU_WAIT_INPUT)
    return;

  if (cpuState == CPU_DELAY)
    return;

  if (cpuState == CPU_STOPPED)
    return;

  if (cpuState == CPU_HALTED)
    return;


  // ----------------------------------------------------------
  // FETCH
  // ----------------------------------------------------------

  byte pcAntes = PC;

  byte opcode = fetch();


  // Mostra no endereço a instrução executada

  addr = pcAntes;


  // ----------------------------------------------------------
  // EXECUTE
  // ----------------------------------------------------------

  executarOpcode(opcode);


  // ----------------------------------------------------------
  // DATA
  // ----------------------------------------------------------

  atualizarDataCPU();
}


// ============================================================
// FETCH
// ============================================================

byte fetch() {

  byte valor = memory[PC];

  PC++;

  return valor;
}


// ============================================================
// FETCH OPERAND
// ============================================================

byte fetchOperand() {

  byte valor = memory[PC];

  PC++;

  return valor;
}


// ============================================================
// EXECUTOR
// ============================================================

void executarOpcode(byte opcode) {

  switch (opcode) {


    // ========================================================
    // SYSTEM
    // ========================================================

    case OP_NOP:

      break;


    case OP_HALT:

      // ------------------------------------------------------
      // IMPORTANTE:
      //
      // HALT agora coloca explicitamente a CPU em HALT.
      // Não volta para STOPPED.
      // ------------------------------------------------------

      cpuState = CPU_HALTED;

      ioWaiting = false;

      Serial.println();
      Serial.println("================================");
      Serial.println(" HALT");
      Serial.println(" CPU em HALT.");
      Serial.println(" Pressione RUN/STOP para sair.");
      Serial.println("================================");

      break;


    // ========================================================
    // LDA
    // ========================================================

    case OP_LDA_MEM: {

      byte endereco = fetchOperand();

      A = memory[endereco];

      atualizaZN(A);

      break;
    }


    case OP_LDA_IMM:

      A = fetchOperand();

      atualizaZN(A);

      break;

    case OP_LDB_MEM: {

      byte endereco = fetchOperand();

      B = memory[endereco];

      atualizaZN(B);

      break;
    }


    case OP_LDB_IMM: {

      B = fetchOperand();

      atualizaZN(B);

      break;

    }

    case OP_LDA_INDB: {

      A = memory[B];

      atualizaZN(A);

      break;
    }

    // ========================================================
    // STA
    // ========================================================

    case OP_STA_MEM: {

      byte endereco = fetchOperand();

      memory[endereco] = A;

      break;
    }

    case OP_STA_INDB: {

      memory[B] = A;

      break;
    }

    case OP_STB_MEM: {

      byte endereco = fetchOperand();

      memory[endereco] = B;

      break;
    }


    // ========================================================
    // MOV
    // ========================================================

    case OP_MOV_A_B:

      B = A;

      atualizaZN(B);

      break;


    case OP_MOV_B_A:

      A = B;

      atualizaZN(A);

      break;


    case OP_SWAP: {

      byte temp = A;

      A = B;
      B = temp;

      atualizaZN(A);

      break;
    }


    // ========================================================
    // ADD
    // ========================================================

    case OP_ADD_B:

      aluADD(B);

      break;


    case OP_ADD_MEM: {

      byte endereco = fetchOperand();

      aluADD(memory[endereco]);

      break;
    }


    case OP_ADD_IMM:

      aluADD(fetchOperand());

      break;


    // ========================================================
    // SUB
    // ========================================================

    case OP_SUB_B:

      aluSUB(B);

      break;


    case OP_SUB_MEM: {

      byte endereco = fetchOperand();

      aluSUB(memory[endereco]);

      break;
    }


    case OP_SUB_IMM:

      aluSUB(fetchOperand());

      break;


    // ========================================================
    // OR
    // ========================================================

    case OP_OR_B:

      A = A | B;

      limpaCV();

      atualizaZN(A);

      break;


    case OP_OR_MEM: {

      byte endereco = fetchOperand();

      A = A | memory[endereco];

      limpaCV();

      atualizaZN(A);

      break;
    }


    // ========================================================
    // AND
    // ========================================================

    case OP_AND_B:

      A = A & B;

      limpaCV();

      atualizaZN(A);

      break;


    case OP_AND_MEM: {

      byte endereco = fetchOperand();

      A = A & memory[endereco];

      limpaCV();

      atualizaZN(A);

      break;
    }


    // ========================================================
    // XOR
    // ========================================================

    case OP_XOR_B:

      A = A ^ B;

      limpaCV();

      atualizaZN(A);

      break;


    case OP_XOR_MEM: {

      byte endereco = fetchOperand();

      A = A ^ memory[endereco];

      limpaCV();

      atualizaZN(A);

      break;
    }

    case OP_XOR_IMM: {
      byte valor = fetchOperand();

      A = A ^ valor;

      limpaCV();

      atualizaZN(A);

      break;
    }


    // ========================================================
    // NOT
    // ========================================================

    case OP_NOT_A:

      A = ~A;

      limpaCV();

      atualizaZN(A);

      break;


    case OP_NOT_B:

      B = ~B;

      limpaCV();

      atualizaZN(B);

      break;


    case OP_AND_IMM: {

      byte valor = fetchOperand();

      A = A & valor;

      limpaCV();

      atualizaZN(A);

      break;
    }


    case OP_OR_IMM: {

      byte valor = fetchOperand();

      A = A | valor;

      limpaCV();

      atualizaZN(A);

      break;
    }


    // ========================================================
    // COMPARE
    // ========================================================

    case OP_CP_B:

      aluCMP(B);

      break;


    case OP_CP_MEM: {

      byte endereco = fetchOperand();

      aluCMP(memory[endereco]);

      break;
    }


    case OP_CP_IMM:

      aluCMP(fetchOperand());

      break;


    // ========================================================
    // INC / DEC
    // ========================================================

    case OP_INC_A:

      aluINC(true);

      break;


    case OP_DEC_A:

      aluDEC(true);

      break;


    case OP_INC_B:

      aluINC(false);

      break;


    case OP_DEC_B:

      aluDEC(false);

      break;


    case OP_INC_MEM: {

      byte endereco = fetchOperand();

      byte antigo = memory[endereco];

      byte resultado = antigo + 1;

      memory[endereco] = resultado;

      if (antigo == 0xFF)
        bitSet(flags, FLAG_C);
      else
        bitClear(flags, FLAG_C);

      if (antigo == 0x7F)
        bitSet(flags, FLAG_V);
      else
        bitClear(flags, FLAG_V);

      atualizaZN(resultado);

      break;
    }


    case OP_DEC_MEM: {

      byte endereco = fetchOperand();

      byte antigo = memory[endereco];

      byte resultado = antigo - 1;

      memory[endereco] = resultado;

      // C = 0 -> borrow

      if (antigo == 0)
        bitClear(flags, FLAG_C);
      else
        bitSet(flags, FLAG_C);

      if (antigo == 0x80)
        bitSet(flags, FLAG_V);
      else
        bitClear(flags, FLAG_V);

      atualizaZN(resultado);

      break;
    }


    // ========================================================
    // JUMPS
    // ========================================================

    case OP_JP: {

      byte endereco = fetchOperand();

      PC = endereco;

      break;
    }


    case OP_JPZ: {

      byte endereco = fetchOperand();

      if (flagAtiva(FLAG_Z))
        PC = endereco;

      break;
    }


    case OP_JPNZ: {

      byte endereco = fetchOperand();

      if (!flagAtiva(FLAG_Z))
        PC = endereco;

      break;
    }


    case OP_JPC: {

      byte endereco = fetchOperand();

      if (flagAtiva(FLAG_C))
        PC = endereco;

      break;
    }


    case OP_JPNC: {

      byte endereco = fetchOperand();

      if (!flagAtiva(FLAG_C))
        PC = endereco;

      break;
    }


    case OP_JPN: {

      byte endereco = fetchOperand();

      if (flagAtiva(FLAG_N))
        PC = endereco;

      break;
    }


    case OP_JNN: {

      byte endereco = fetchOperand();

      if (!flagAtiva(FLAG_N))
        PC = endereco;

      break;
    }


    case OP_JPV: {

      byte endereco = fetchOperand();

      if (flagAtiva(FLAG_V))
        PC = endereco;

      break;
    }


    case OP_JPNV: {

      byte endereco = fetchOperand();

      if (!flagAtiva(FLAG_V))
        PC = endereco;

      break;
    }


    // ========================================================
    // SUBROTINAS
    // ========================================================

    case OP_CALL: {

      byte endereco = fetchOperand();

      // Guarda o endereço de retorno (PC já aponta para a
      // instrução seguinte, pois fetchOperand() avançou o PC).

      memory[RET_ADDR] = PC;

      PC = endereco;

      break;
    }


    case OP_RET: {

      PC = memory[RET_ADDR];

      break;
    }


    // ========================================================
    // SHIFT / ROTATE
    // ========================================================

    case OP_SHL_A:

      shiftLeft(true);

      break;


    case OP_SHR_A:

      shiftRight(true);

      break;


    case OP_ROL_A:

      rotateLeft(true);

      break;


    case OP_ROR_A:

      rotateRight(true);

      break;


    case OP_SHL_B:

      shiftLeft(false);

      break;


    case OP_SHR_B:

      shiftRight(false);

      break;


    case OP_ROL_B:

      rotateLeft(false);

      break;


    case OP_ROR_B:

      rotateRight(false);

      break;


    // ========================================================
    // I/O
    // ========================================================

    case OP_INA:

      iniciarINA();

      break;


    case OP_OUTA:

      executarOUTA();

      break;


    case OP_KEY:

      executarKEY();

      break;


    // ========================================================
    // RANDOM
    // ========================================================

    case OP_RNDA:

      A = random(0, 256);

      atualizaZN(A);

      break;


    // ========================================================
    // DELAY
    // ========================================================

    case OP_DLAYA:

      delayInicio = millis();

      delayDuracao = (unsigned long)A;

      delayPorStep = executandoStep;

      cpuState = CPU_DELAY;

      break;

    case OP_DLAYB:

      delayInicio = millis();

      delayDuracao = (unsigned long)B;

      delayPorStep = executandoStep;

      cpuState = CPU_DELAY;

      break;


    // ========================================================
    // OPCODE INVÁLIDO
    // ========================================================

    default:

      Serial.print("ERRO: opcode desconhecido 0x");

      if (opcode < 16)
        Serial.print("0");

      Serial.println(opcode, HEX);

      cpuState = CPU_HALTED;

      break;
  }
}


// ============================================================
// ALU ADD
// ============================================================

void aluADD(byte valor) {

  byte antigoA = A;

  uint16_t resultado =
    (uint16_t)A + (uint16_t)valor;

  A = (byte)resultado;


  // Carry

  if (resultado > 255)
    bitSet(flags, FLAG_C);
  else
    bitClear(flags, FLAG_C);


  // Overflow

  bool sinalA = (antigoA & 0x80) != 0;
  bool sinalB = (valor   & 0x80) != 0;
  bool sinalR = (A       & 0x80) != 0;


  if ((sinalA == sinalB) &&
      (sinalA != sinalR)) {

    bitSet(flags, FLAG_V);

  } else {

    bitClear(flags, FLAG_V);
  }


  atualizaZN(A);
}


// ============================================================
// ALU SUB
// ============================================================

void aluSUB(byte valor) {

  byte antigoA = A;

  uint16_t resultado =
    (uint16_t)A - (uint16_t)valor;

  A = (byte)resultado;


  // C = 1 → não houve borrow
  // C = 0 → houve borrow

  if (antigoA >= valor)
    bitSet(flags, FLAG_C);
  else
    bitClear(flags, FLAG_C);


  // Overflow

  bool sinalA = (antigoA & 0x80) != 0;
  bool sinalB = (valor   & 0x80) != 0;
  bool sinalR = (A       & 0x80) != 0;


  if ((sinalA != sinalB) &&
      (sinalA != sinalR)) {

    bitSet(flags, FLAG_V);

  } else {

    bitClear(flags, FLAG_V);
  }


  atualizaZN(A);
}


// ============================================================
// ALU CMP
// ============================================================

void aluCMP(byte valor) {

  byte antigoA = A;

  uint16_t resultado =
    (uint16_t)A - (uint16_t)valor;

  byte resultado8 =
    (byte)resultado;


  // Carry

  if (antigoA >= valor)
    bitSet(flags, FLAG_C);
  else
    bitClear(flags, FLAG_C);


  // Overflow

  bool sinalA = (antigoA    & 0x80) != 0;
  bool sinalB = (valor      & 0x80) != 0;
  bool sinalR = (resultado8 & 0x80) != 0;


  if ((sinalA != sinalB) &&
      (sinalA != sinalR)) {

    bitSet(flags, FLAG_V);

  } else {

    bitClear(flags, FLAG_V);
  }


  atualizaZN(resultado8);
}


// ============================================================
// INC
// ============================================================

void aluINC(bool registroA) {

  byte antigo;

  if (registroA)
    antigo = A;
  else
    antigo = B;


  byte resultado =
    antigo + 1;


  if (registroA)
    A = resultado;
  else
    B = resultado;


  if (antigo == 0xFF)
    bitSet(flags, FLAG_C);
  else
    bitClear(flags, FLAG_C);


  if (antigo == 0x7F)
    bitSet(flags, FLAG_V);
  else
    bitClear(flags, FLAG_V);


  atualizaZN(resultado);
}


// ============================================================
// DEC
// ============================================================

void aluDEC(bool registroA) {

  byte antigo;

  if (registroA)
    antigo = A;
  else
    antigo = B;


  byte resultado =
    antigo - 1;


  if (registroA)
    A = resultado;
  else
    B = resultado;


  // C = 0 → borrow

  if (antigo == 0)
    bitClear(flags, FLAG_C);
  else
    bitSet(flags, FLAG_C);


  if (antigo == 0x80)
    bitSet(flags, FLAG_V);
  else
    bitClear(flags, FLAG_V);


  atualizaZN(resultado);
}


// ============================================================
// ZERO / NEGATIVE
// ============================================================

void atualizaZN(byte valor) {

  if (valor == 0)
    bitSet(flags, FLAG_Z);
  else
    bitClear(flags, FLAG_Z);


  if (valor & 0x80)
    bitSet(flags, FLAG_N);
  else
    bitClear(flags, FLAG_N);
}


// ============================================================
// LIMPA C/V
// ============================================================

void limpaCV() {

  bitClear(flags, FLAG_C);
  bitClear(flags, FLAG_V);
}


// ============================================================
// TESTA FLAG
// ============================================================

bool flagAtiva(byte flag) {

  return bitRead(flags, flag);
}


// ============================================================
// SHIFT LEFT
// ============================================================

void shiftLeft(bool registroA) {

  byte valor;

  if (registroA)
    valor = A;
  else
    valor = B;


  if (valor & 0x80)
    bitSet(flags, FLAG_C);
  else
    bitClear(flags, FLAG_C);


  valor <<= 1;


  if (registroA)
    A = valor;
  else
    B = valor;


  atualizaZN(valor);
}


// ============================================================
// SHIFT RIGHT
// ============================================================

void shiftRight(bool registroA) {

  byte valor;

  if (registroA)
    valor = A;
  else
    valor = B;


  if (valor & 0x01)
    bitSet(flags, FLAG_C);
  else
    bitClear(flags, FLAG_C);


  valor >>= 1;


  if (registroA)
    A = valor;
  else
    B = valor;


  atualizaZN(valor);
}


// ============================================================
// ROTATE LEFT
// ============================================================

void rotateLeft(bool registroA) {

  byte valor;

  if (registroA)
    valor = A;
  else
    valor = B;


  bool bit7 =
    (valor & 0x80) != 0;


  valor <<= 1;


  if (bit7)
    valor |= 0x01;


  if (bit7)
    bitSet(flags, FLAG_C);
  else
    bitClear(flags, FLAG_C);


  if (registroA)
    A = valor;
  else
    B = valor;


  atualizaZN(valor);
}


// ============================================================
// ROTATE RIGHT
// ============================================================

void rotateRight(bool registroA) {

  byte valor;

  if (registroA)
    valor = A;
  else
    valor = B;


  bool bit0 =
    (valor & 0x01) != 0;


  valor >>= 1;


  if (bit0)
    valor |= 0x80;


  if (bit0)
    bitSet(flags, FLAG_C);
  else
    bitClear(flags, FLAG_C);


  if (registroA)
    A = valor;
  else
    B = valor;


  atualizaZN(valor);
}


// ============================================================
// INA
// ============================================================

void iniciarINA() {

  ioWaiting = true;
  ioPorStep = executandoStep;

  cpuState = CPU_WAIT_INPUT;


  Serial.println();
  Serial.println("INA: aguardando entrada...");
  Serial.println("Pressione INCR para confirmar.");
}


// ============================================================
// OUTA
// ============================================================

void executarOUTA() {

  //data = A;
  ledOutSend(A);

  Serial.print("OUTA: 0x");

  if (A < 16)
    Serial.print("0");

  Serial.println(A, HEX);
}


// ============================================================
// KEY
// ============================================================

void executarKEY() {
  digitalWrite(clockPin, HIGH);


  digitalWrite(latchPin, LOW);

  delayMicroseconds(5);

  digitalWrite(latchPin, HIGH);


  data =
    shiftIn(
      dataPin,
      clockPin,
      MSBFIRST
    );

  A = ~data;

  atualizaZN(A);
}


// ============================================================
// PROCESSA DELAY
// ============================================================

void processarDelay() {

  if (millis() - delayInicio >= delayDuracao) {

    if (delayPorStep) {

      // O DELAY foi iniciado por STEP.
      // Portanto, o STEP termina aqui.
      cpuState = CPU_STOPPED;

    } else {

      // O DELAY foi iniciado durante RUN.
      // Continua executando normalmente.
      cpuState = CPU_RUNNING;
    }

    delayPorStep = false;

    return;
  }
}


// ============================================================
// RESET CPU
// ============================================================

void resetCPU() {

  A = 0;
  B = 0;
  PC = 0;

  flags = 0;

  ioWaiting = false;

  memMode = false;

  cpuState = CPU_STOPPED;

  delayInicio = 0;
  delayDuracao = 0;

  stepPulse = false;


  addr = 0;
  data = 0;
  controle = 0;

  ledOutSend(0);
}


// ============================================================
// BOTÕES
// ============================================================

void leEntradasControles() {

  for (byte i = 0; i < totalPinos; i++) {

    int estadoAtual =
      digitalRead(pinosEntrada[i]);


    // --------------------------------------------------------
    // Detecta borda HIGH → LOW
    // --------------------------------------------------------

    if (estadosAnteriores[i] == HIGH &&
        estadoAtual == LOW) {


      Serial.print("Botao: ");
      Serial.println(nomesPinos[i]);


      switch (pinosEntrada[i]) {


        // ====================================================
        // STEP
        // ====================================================

        case inStep:
          // --------------------------------------------------
          // MEMORY MANAGER CLR(hold)+STEP
          // --------------------------------------------------
          if (digitalRead(inClr) == LOW) {
            loadSaveMemory();
            resetCPU();
            addr = 0;
            data = memory[addr];
            atualizar595();
            break;
          }

          // --------------------------------------------------
          // HALT
          // --------------------------------------------------

          if (cpuState == CPU_HALTED) {

            Serial.println("CPU em HALT.");
            Serial.println("Use RUN/STOP para sair.");

            break;
          }


          // --------------------------------------------------
          // WAIT INPUT
          // --------------------------------------------------

          if (cpuState == CPU_WAIT_INPUT) {

            Serial.println("CPU aguardando INA.");

            break;
          }


          // --------------------------------------------------
          // STOPPED
          // --------------------------------------------------

          if (cpuState == CPU_STOPPED) {

            stepPulse = true;

            stepPulseInicio =
              millis();


            cpuState =
              CPU_RUNNING;


            executandoStep = true;

            cpuStep();

            executandoStep = false;


            // ------------------------------------------------
            // Se a instrução não mudou o estado,
            // retorna para STOPPED.
            // ------------------------------------------------

            if (cpuState == CPU_RUNNING) {

              cpuState =
                CPU_STOPPED;
            }
          }

          break;


        // ====================================================
        // ADDR
        // ====================================================
        //
        // DATA → ADDR
        //
        // E agora também:
        //
        // DATA → PC
        //
        // Isso permite:
        //
        // CLR
        // ADDR
        // STEP
        //
        // começar a execução em 0x00.
        //
        // ====================================================

        case inAddr:

          if (cpuState == CPU_STOPPED) {

            addr = data;

            // ------------------------------------------------
            // CORREÇÃO IMPORTANTE:
            // sincroniza o PC com o endereço selecionado.
            // ------------------------------------------------

            PC = addr;

            data = memory[addr];


            Serial.print("ADDR = 0x");

            if (addr < 16)
              Serial.print("0");

            Serial.print(addr, HEX);

            Serial.print(" | PC = 0x");

            if (PC < 16)
              Serial.print("0");

            Serial.println(PC, HEX);
          }

          else if (cpuState == CPU_HALTED) {

            Serial.println("CPU em HALT.");
            Serial.println("Use RUN/STOP para sair antes de editar.");
          }

          break;


        // ====================================================
        // RUN / STOP
        // ====================================================

        case inRun:


          // --------------------------------------------------
          // RUNNING → STOPPED
          // --------------------------------------------------

          if (cpuState == CPU_RUNNING) {

            cpuState =
              CPU_STOPPED;

            ioWaiting = false;

            Serial.println("CPU STOP.");
          }


          // --------------------------------------------------
          // STOPPED → RUNNING
          // --------------------------------------------------

          else if (cpuState == CPU_STOPPED) {

            cpuState =
              CPU_RUNNING;

            Serial.println("CPU RUN.");
          }


          // --------------------------------------------------
          // HALTED → STOPPED
          // --------------------------------------------------
          //
          // Este é o novo comportamento.
          //
          // HALT não precisa de RESET.
          //
          // RUN/STOP libera a CPU e volta para programação.
          //
          // --------------------------------------------------

          else if (cpuState == CPU_HALTED) {

            cpuState =
              CPU_STOPPED;

            ioWaiting = false;

            Serial.println();
            Serial.println("==============================");
            Serial.println(" SAINDO DE HALT");
            Serial.println(" CPU em modo de programacao.");
            Serial.println("==============================");
          }


          // --------------------------------------------------
          // WAIT INPUT
          // --------------------------------------------------

          else if (cpuState == CPU_WAIT_INPUT) {

            cpuState =
              CPU_STOPPED;

            ioWaiting = false;

            Serial.println("CPU aguardando entrada.");
            Serial.println("Use INCR para confirmar.");
          }


          // --------------------------------------------------
          // DELAY
          // --------------------------------------------------

          else if (cpuState == CPU_DELAY) {
            cpuState =
              CPU_STOPPED;

            ioWaiting = false;

            Serial.println("CPU está em DELAY.");
          }

          break;


        // ====================================================
        // CLEAR
        // ====================================================

        case inClr:

          if (cpuState == CPU_STOPPED) {

            data = 0;

            Serial.println("DATA = 0x00");
          }

          else if (cpuState == CPU_HALTED) {

            Serial.println("CPU em HALT.");
            Serial.println("Use RUN/STOP para voltar a programacao.");
          }

          break;


        // ====================================================
        // INCR
        // ====================================================

        case inIncr:


          // --------------------------------------------------
          // INA esperando entrada
          // --------------------------------------------------

          if (cpuState == CPU_WAIT_INPUT) {

            A = data;

            atualizaZN(A);

            ioWaiting = false;

            if (ioPorStep) {
              cpuState = CPU_STOPPED;
            } else {
              cpuState = CPU_RUNNING;
            }

            ioPorStep = false;

            Serial.print("INA recebeu: 0x");

            if (A < 16)
              Serial.print("0");

            Serial.println(A, HEX);
          }


          // --------------------------------------------------
          // Programação manual
          // --------------------------------------------------

          else if (cpuState == CPU_STOPPED) {

            memory[addr] =
              data;


            Serial.print("MEM[0x");

            if (addr < 16)
              Serial.print("0");

            Serial.print(addr, HEX);

            Serial.print("] = 0x");

            if (data < 16)
              Serial.print("0");

            Serial.println(data, HEX);


            addr++;

            PC = addr;

            data =
              memory[addr];
          }

          break;
      }
    }


    estadosAnteriores[i] =
      estadoAtual;
  }
}


// ============================================================
// LE 8 BITS
// ============================================================

void leEntrada8bits() {

  digitalWrite(clockPin, HIGH);


  digitalWrite(latchPin, LOW);

  delayMicroseconds(5);

  digitalWrite(latchPin, HIGH);


  dadosBotoes =
    shiftIn(
      dataPin,
      clockPin,
      MSBFIRST
    );


  for (byte i = 0; i < 8; i++) {

    byte estadoAtualBit =
      bitRead(dadosBotoes, i);

    byte estadoAnteriorBit =
      bitRead(estadoAnteriorBotoes, i);


    if (estadoAnteriorBit == 1 &&
        estadoAtualBit == 0) {


      bool bitAtualData =
        bitRead(data, i);


      bitWrite(
        data,
        i,
        !bitAtualData
      );
    }
  }


  estadoAnteriorBotoes =
    dadosBotoes;
}


// ============================================================
// DATA DA CPU
// ============================================================

void atualizarDataCPU() {

  data = A;
}


// ============================================================
// CONTROLE
// ============================================================

void atualizarControle() {

  controle = 0;


  // ----------------------------------------------------------
  // MEM
  // ----------------------------------------------------------

  if (memMode)
    bitSet(controle, CTRL_MEM);


  // ----------------------------------------------------------
  // RUN
  // ----------------------------------------------------------

  if (cpuState == CPU_RUNNING ||
      stepPulse) {

    bitSet(controle, CTRL_RUN);
  }


  // ----------------------------------------------------------
  // I/O
  // ----------------------------------------------------------

  if (ioWaiting)
    bitSet(controle, CTRL_IO);


  // ----------------------------------------------------------
  // HALT
  // ----------------------------------------------------------

  if (cpuState == CPU_HALTED)
    bitSet(controle, CTRL_HALT);


  // ----------------------------------------------------------
  // FLAGS
  // ----------------------------------------------------------

  if (flagAtiva(FLAG_V))
    bitSet(controle, CTRL_V);


  if (flagAtiva(FLAG_C))
    bitSet(controle, CTRL_C);


  if (flagAtiva(FLAG_Z))
    bitSet(controle, CTRL_Z);


  if (flagAtiva(FLAG_N))
    bitSet(controle, CTRL_N);
}


// ============================================================
// ATUALIZA 74HC595
// ============================================================

void atualizar595() {

  atualizarControle();


  digitalWrite(LATCH_PIN, LOW);


  shiftOut(
    DATA_PIN,
    CLOCK_PIN,
    MSBFIRST,
    ~controle
  );


  shiftOut(
    DATA_PIN,
    CLOCK_PIN,
    MSBFIRST,
    ~data
  );


  shiftOut(
    DATA_PIN,
    CLOCK_PIN,
    MSBFIRST,
    ~addr
  );


  digitalWrite(LATCH_PIN, HIGH);
}


// ============================================================
// DEBUG CPU
// ============================================================

void debugCPU() {

  Serial.println();
  Serial.println("========== CPU ==========");


  Serial.print("A  = 0x");

  if (A < 16)
    Serial.print("0");

  Serial.println(A, HEX);


  Serial.print("B  = 0x");

  if (B < 16)
    Serial.print("0");

  Serial.println(B, HEX);


  Serial.print("PC = 0x");

  if (PC < 16)
    Serial.print("0");

  Serial.println(PC, HEX);


  Serial.print("FLAGS = ");

  Serial.print("N=");
  Serial.print(flagAtiva(FLAG_N));

  Serial.print(" Z=");
  Serial.print(flagAtiva(FLAG_Z));

  Serial.print(" C=");
  Serial.print(flagAtiva(FLAG_C));

  Serial.print(" V=");
  Serial.println(flagAtiva(FLAG_V));


  Serial.print("STATE = ");


  switch (cpuState) {

    case CPU_STOPPED:
      Serial.println("STOPPED");
      break;

    case CPU_RUNNING:
      Serial.println("RUNNING");
      break;

    case CPU_WAIT_INPUT:
      Serial.println("WAIT_INPUT");
      break;

    case CPU_HALTED:
      Serial.println("HALTED");
      break;

    case CPU_DELAY:
      Serial.println("DELAY");
      break;
  }


  Serial.println("=========================");
}


// ============================================================
// DEBUG MEMÓRIA
// ============================================================

void debugMemory() {

  Serial.println();
  Serial.println("--- DUMP DE MEMÓRIA ---");


  for (int i = 0; i < 256; i += 16) {

    if (i < 16)
      Serial.print("0");

    Serial.print(i, HEX);

    Serial.print(": ");


    for (int j = 0; j < 16; j++) {

      byte dado =
        memory[i + j];


      if (dado < 16)
        Serial.print("0");

      Serial.print(dado, HEX);

      Serial.print(" ");
    }


    Serial.println();
  }


  Serial.println("-----------------------");
}


// ============================================================
// FIM
// ============================================================
