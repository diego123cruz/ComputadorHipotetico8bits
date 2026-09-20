// ============================================================
// TAPE.INO — RECEPTOR DE FITA POR ÁUDIO
// Computador Hipotético 8 bits
//
// Arduino Nano / ATmega328P
//
// Recebe, via um pino digital, o áudio gerado pelo "tape.html"
// (FSK 1kHz/2kHz, mesmo padrão de bits do carregador k7.z80 do
// projeto Z80) e reconstrói os bytes originais na memória.
//
// Versão enxuta: as versões anteriores tinham bastante
// instrumentação de depuração (logs, buffers de pulsos crus) que
// ajudou a confirmar que o sinal físico está limpo — mas também
// consumiu tanta RAM do Nano (75% do total, deixando só ~500
// bytes livres) que o próprio Arduino ficava instável, dando
// contagens de bit erráticas sem relação com o sinal real. Esta
// versão volta ao essencial, com só um contador leve de erro por
// bit (sem arrays grandes), para não repetir esse problema.
//
// Diego — 2026
// ============================================================
//
// ------------------------------------------------------------
// CIRCUITO DE ENTRADA (o "ouvido" do Arduino)
// ------------------------------------------------------------
// OPÇÃO A — módulo pronto com comparador (ex.: KY-038 com LM393,
// só 3 pinos: 5V, GND, OUT):
//
//   KY-038 OUT --------- pino digital TAPE_PIN
//   KY-038 5V  --------- +5V
//   KY-038 GND --------- GND
//
//   Ajuste o trimpot do módulo até o LED dele acompanhar o tom.
//
// OPÇÃO B — ligação direta por fio a partir de uma saída de
// áudio (P2, fone de ouvido etc.), sem módulo — a opção mais
// confiável, já testada com sucesso neste projeto:
//
//   P2 (saída de áudio) --[C1]-- nó X --[pino digital TAPE_PIN]
//                                  |
//                          R1 (10k) para +5V
//                                  |
//                          R2 (10k) para GND
//
//   C1 = 1uF (cerâmico ou eletrolítico, bloqueia DC)
//   R1 = R2 = 10k (divisor formando o "meio da escala", ~2.5V)
//
// ============================================================

// ============================================================
// CONFIGURAÇÃO
// ============================================================

#define TAPE_PIN     8      // pino digital ligado ao circuito acima
#define LED_PIN      13     // LED onboard, usado como indicador

// Modo de calibração/diagnóstico: em vez de tentar decodificar,
// só mede e imprime a duração de cada pulso recebido no pino, sem
// parar. Use isso primeiro com qualquer módulo/circuito novo.
#define MODO_DIAGNOSTICO false

// Larguras de pulso esperadas (meio-ciclo, em microssegundos).
// O tape.html gera onda quadrada com F1=1000Hz e F2=2000Hz, ou
// seja, meio-ciclo de 500us (1kHz) e 250us (2kHz).
#define SHORT_US     250UL   // meio-ciclo @2kHz ("curto")
#define LONG_US      500UL   // meio-ciclo @1kHz ("longo")
#define THRESH_US    450UL   // limiar entre curto e longo (calibrado com hardware real)

// Quantos ciclos curtos/longos formam cada bit (mesmos valores
// usados no tape.html / k7.z80): ambos os bits duram o mesmo
// tempo total, só muda a proporção entre as duas frequências.
#define ZERO_SHORTS  8
#define ZERO_LONGS   2
#define UM_SHORTS    4
#define UM_LONGS     4

// Tempo máximo de espera por um pulso antes de considerar que
// o sinal sumiu (silêncio / fim de transmissão / erro).
#define PULSE_TIMEOUT_US   5000UL

// Quantos meios-ciclos longos seguidos são necessários para dar
// como "sincronizado" no tom de sincronismo (leader) de 1kHz.
#define LEADER_LOCK_CYCLES 100

// Quantos pulsos curtos SEGUIDOS exigimos antes de aceitar que o
// leader terminou. O bit de start é sempre 0 (sempre 8 curtos
// seguidos), então dá pra exigir até 6 sem risco de rejeitar uma
// transição de verdade — e isso filtra ruído/solavanco isolado.
#define LEADER_FIM_CONFIRMACOES 6

#define MEM_SIZE 256

// ============================================================
// MEMÓRIA RECEBIDA
// ============================================================
byte memory[MEM_SIZE];

// ============================================================
// LEITURA DE PULSOS
// ============================================================

unsigned long medirPulso() {
  return pulseIn(TAPE_PIN, HIGH, PULSE_TIMEOUT_US);
}

bool ehCurto(unsigned long us) {
  return us > 0 && us < THRESH_US;
}

bool ehLongo(unsigned long us) {
  return us >= THRESH_US;
}

// ------------------------------------------------------------
// Fila pequena de pulsos "devolvidos" (só usada na transição
// entre o leader e o primeiro bit do cabeçalho).
// ------------------------------------------------------------

#define PENDENTES_MAX 8
unsigned long filaPendente[PENDENTES_MAX];
int pendenteQtd = 0;
int pendentePos = 0;

unsigned long proximoPulso() {
  if (pendentePos < pendenteQtd) {
    return filaPendente[pendentePos++];
  }
  return medirPulso();
}

void devolverVarios(unsigned long *valores, int n) {
  for (int i = 0; i < n && i < PENDENTES_MAX; i++) filaPendente[i] = valores[i];
  pendenteQtd = n;
  pendentePos = 0;
}

// ============================================================
// DECODIFICAÇÃO DE BITS / BYTES
// ============================================================

// Guarda, de forma leve (sem array grande), qual foi o último bit
// lido — rótulo + contagem de curtos — só pra dar uma pista de
// diagnóstico quando um byte falha, sem gastar RAM com buffers.
const char* ultimoRotulo = "";
int ultimoCurtos = -1;

int lerBit(const char* rotulo) {
  int curtos = 0;
  unsigned long p;

  while (true) {
    p = proximoPulso();
    if (p == 0) {
      ultimoRotulo = rotulo;
      ultimoCurtos = curtos;
      return -1; // timeout: sinal sumiu
    }
    if (ehCurto(p)) {
      curtos++;
    } else {
      break; // pulso longo: fim da rajada curta deste bit
    }
  }

  int bit = (curtos >= 6) ? 0 : 1;

  int totalLongos = (bit == 0) ? ZERO_LONGS : UM_LONGS;
  int faltam = totalLongos - 1;
  for (int i = 0; i < faltam; i++) {
    p = proximoPulso();
    if (p == 0) {
      ultimoRotulo = rotulo;
      ultimoCurtos = curtos;
      return -1;
    }
  }

  ultimoRotulo = rotulo;
  ultimoCurtos = curtos;
  return bit;
}

#define ERRO_START  -1
#define ERRO_BIT    -2
#define ERRO_STOP   -3

const char* descreverErro(int codigo) {
  switch (codigo) {
    case ERRO_START: return "start bit veio != 0";
    case ERRO_BIT:    return "timeout no meio do byte";
    case ERRO_STOP:   return "stop bit veio != 1";
    default:          return "erro desconhecido";
  }
}

int lerByte() {
  int start = lerBit("start");
  if (start != 0) return ERRO_START;

  int valor = 0;
  static const char* rotulosDado[8] = {"dado0","dado1","dado2","dado3","dado4","dado5","dado6","dado7"};
  for (int i = 0; i < 8; i++) {
    int b = lerBit(rotulosDado[i]);
    if (b < 0) return ERRO_BIT;
    if (b) valor |= (1 << i);
  }

  int stop = lerBit("stop");
  if (stop != 1) return ERRO_STOP;

  return valor;
}

bool esperarLeader() {
  int longosSeguidos = 0;
  unsigned long p;

  while (true) {
    p = medirPulso();
    if (p == 0) { longosSeguidos = 0; continue; }
    if (ehLongo(p)) {
      longosSeguidos++;
      if (longosSeguidos >= LEADER_LOCK_CYCLES) break;
    } else {
      longosSeguidos = 0;
    }
  }

  unsigned long confirmacao[LEADER_FIM_CONFIRMACOES];
  int confirmadas = 0;

  while (true) {
    p = medirPulso();
    if (p == 0) return false;

    if (ehCurto(p)) {
      confirmacao[confirmadas++] = p;
      if (confirmadas >= LEADER_FIM_CONFIRMACOES) {
        devolverVarios(confirmacao, confirmadas);
        return true;
      }
    } else {
      confirmadas = 0;
    }
  }
}

// ============================================================
// ROTINA PRINCIPAL DE CARGA
// ============================================================
//
// Cabeçalho (7 bytes): título(2) + endereço inicial(2) +
// endereço final(2, inclusive) + checksum(1).

struct ResultadoCarga {
  bool ok;
  byte titulo0, titulo1;
  int enderecoInicial, enderecoFinal;
  byte checksumEsperado, checksumCalculado;
};

// Variável global em vez de retornar o struct por valor: o
// compilador do Arduino gera um protótipo de carregarFita() e o
// insere no TOPO do arquivo, antes desta struct existir. Uma
// função retornando bool evita esse problema.
ResultadoCarga resultado;

bool carregarFita() {
  resultado.ok = false;

  Serial.println();
  Serial.println(F("Aguardando tom de sincronismo (leader)..."));
  digitalWrite(LED_PIN, LOW);

  if (!esperarLeader()) {
    Serial.println(F("ERRO: nao sincronizou no leader."));
    return false;
  }

  Serial.println(F("Sincronizado! Lendo cabecalho..."));
  digitalWrite(LED_PIN, HIGH);

  byte cabecalho[7];
  for (int i = 0; i < 7; i++) {
    int b = lerByte();
    if (b < 0) {
      Serial.print(F("ERRO no cabecalho, byte "));
      Serial.print(i);
      Serial.print(F(" -> "));
      Serial.print(descreverErro(b));
      Serial.print(F(" (ultimo bit: "));
      Serial.print(ultimoRotulo);
      Serial.print(F(", "));
      Serial.print(ultimoCurtos);
      Serial.println(F(" curtos)"));
      return false;
    }
    cabecalho[i] = (byte)b;
  }

  resultado.titulo0 = cabecalho[0];
  resultado.titulo1 = cabecalho[1];
  resultado.enderecoInicial = cabecalho[2] | (cabecalho[3] << 8);
  resultado.enderecoFinal   = cabecalho[4] | (cabecalho[5] << 8);
  resultado.checksumEsperado = cabecalho[6];

  Serial.print(F("Titulo: "));
  Serial.write(resultado.titulo0); Serial.write(resultado.titulo1); Serial.println();
  Serial.print(F("Endereco inicial: 0x"));
  Serial.println(resultado.enderecoInicial, HEX);
  Serial.print(F("Endereco final:   0x"));
  Serial.println(resultado.enderecoFinal, HEX);

  if (resultado.enderecoInicial > resultado.enderecoFinal ||
      resultado.enderecoFinal >= MEM_SIZE) {
    Serial.println(F("ERRO: faixa de enderecos invalida."));
    return false;
  }

  Serial.println(F("Lendo dados..."));
  byte soma = 0;
  for (int addr = resultado.enderecoInicial; addr <= resultado.enderecoFinal; addr++) {
    int b = lerByte();
    if (b < 0) {
      Serial.print(F("ERRO nos dados, endereco 0x"));
      Serial.print(addr, HEX);
      Serial.print(F(" -> "));
      Serial.println(descreverErro(b));
      return false;
    }
    memory[addr] = (byte)b;
    soma = (byte)(soma + b);

    if ((addr & 0x0F) == 0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  resultado.checksumCalculado = soma;

  if (resultado.checksumCalculado != resultado.checksumEsperado) {
    Serial.print(F("ERRO: checksum nao confere. esperado=0x"));
    Serial.print(resultado.checksumEsperado, HEX);
    Serial.print(F(" calculado=0x"));
    Serial.println(resultado.checksumCalculado, HEX);
    return false;
  }

  resultado.ok = true;
  return true;
}

void imprimirMemoria(int inicio, int fim) {
  for (int base = inicio & 0xF0; base <= fim; base += 16) {
    if (base < 16) Serial.print('0');
    Serial.print(base, HEX);
    Serial.print(F(":  "));
    for (int j = 0; j < 16; j++) {
      int addr = base + j;
      if (addr < inicio || addr > fim) { Serial.print(F(".. ")); continue; }
      if (memory[addr] < 16) Serial.print('0');
      Serial.print(memory[addr], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}

// ============================================================
// LED — feedback visual
// ============================================================

void piscarSucesso() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(150);
    digitalWrite(LED_PIN, LOW);  delay(150);
  }
  digitalWrite(LED_PIN, HIGH);
}

void piscarErro() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, HIGH); delay(60);
    digitalWrite(LED_PIN, LOW);  delay(60);
  }
}

// ============================================================
// DIAGNÓSTICO
// ============================================================

void diagnostico() {
  unsigned long p = medirPulso();
  if (p == 0) {
    Serial.println(F("(sem sinal)"));
    return;
  }
  Serial.print(p);
  Serial.print(F("us  -> "));
  if (ehCurto(p)) Serial.println(F("CURTO (2kHz esperado)"));
  else if (ehLongo(p)) Serial.println(F("LONGO (1kHz esperado)"));
  else Serial.println(F("?"));
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(9600);
  pinMode(TAPE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F(" TAPE 8 BITS - receptor de audio"));
  Serial.println(F("=============================="));
  Serial.print(F("Escutando no pino "));
  Serial.println(TAPE_PIN);
#if MODO_DIAGNOSTICO
  Serial.println(F("MODO DIAGNOSTICO: so imprimindo pulsos, sem decodificar."));
#else
  Serial.println(F("De o play no tape.html quando quiser."));
#endif
}

void loop() {
#if MODO_DIAGNOSTICO
  diagnostico();
  return;
#endif

  bool ok = carregarFita();

  if (ok) {
    Serial.println(F(">> CARGA CONCLUIDA COM SUCESSO <<"));
    imprimirMemoria(resultado.enderecoInicial, resultado.enderecoFinal);
    piscarSucesso();
  } else {
    Serial.println(F(">> FALHA NA CARGA <<"));
    piscarErro();
  }

  Serial.println(F("---"));
  delay(500);
}
