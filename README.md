# Computador Hipotético 8 bits

Um computador didático de 8 bits, com CPU, memória e painel de LEDs
implementados em hardware (Arduino Uno/Nano), acompanhado de um
**assembler + emulador** que roda inteiramente no navegador.

---

## Visão geral

| | |
|---|---|
| Registradores | `A`, `B` (8 bits cada) |
| Contador de programa | `PC` (8 bits) |
| Memória | 256 bytes, compartilhada entre dados e programa |
| Flags | `N` (negativo), `Z` (zero), `C` (carry), `V` (overflow) |
| Barramentos | Endereço (8 bits) e Dados (8 bits) |
| E/S | 8 LEDs de saída (`OUTA`) + 8 botões de entrada |

O hardware físico é um Arduino que implementa o fetch-decode-execute
dessa CPU (`ComputadorHipotetico8bits.ino`), acende os LEDs do painel via
registradores de deslocamento (74HC165 / 74HC595) e lê os botões da
mesma forma. O arquivo `computador-hipotetico-8bits.html` reproduz essa
mesma máquina em software, para escrever e testar programas sem precisar
do hardware ligado.

## Painel

| LED | Cor | Representa |
|---|---|---|
| Endereço | Amarelo | Byte do barramento de endereço (`PC` durante execução) |
| Dados | Verde | Byte do barramento de dados (`A` durante execução, ou o valor sendo programado manualmente) |
| Controle / Flags | Vermelho | `N`, `Z`, `C`, `V`, `HALT`, `I/O`, `RUN`, `MEM` |
| Saída (OUTA) | Roxo | Último valor enviado por `OUTA` |

Botões físicos: `STEP`, `ADDR`, `CLR`, `RUN/STOP`, `INCR`, `RESET`, além
das 8 chaves de dados usadas para digitar bytes de programa ou responder
a `INA`.

## Mapa de opcodes

| Hex | Instrução | Operação |
|---|---|---|
| `00` | `NOP` | — |
| `01` | `HALT` | para a CPU |
| `10` | `LDA [addr]` | `A <- MEM[addr]` |
| `11` | `LDA #value` | `A <- value` |
| `12` | `LDB [addr]` | `B <- MEM[addr]` |
| `13` | `LDB #value` | `B <- value` |
| `14` | `LDA [B]` | `A <- MEM[B]` (indireto via B) |
| `20` | `STA [addr]` | `MEM[addr] <- A` |
| `21` | `STA [B]` | `MEM[B] <- A` (indireto via B) |
| `22` | `STB [addr]` | `MEM[addr] <- B` |
| `30` | `MOV A,B` | `B <- A` |
| `31` | `MOV B,A` | `A <- B` |
| `32` | `SWAP` | `A <-> B` |
| `40`/`41`/`42` | `ADD B` / `ADD [addr]` / `ADD #value` | `A <- A + operando` |
| `50`/`51`/`52` | `SUB B` / `SUB [addr]` / `SUB #value` | `A <- A - operando` |
| `60`/`61`/`6A` | `OR B` / `OR [addr]` / `OR #value` | `A <- A OR operando` |
| `62`/`63`/`69` | `AND B` / `AND [addr]` / `AND #value` | `A <- A AND operando` |
| `64`/`65`/`66` | `XOR B` / `XOR [addr]` / `XOR #value` | `A <- A XOR operando` |
| `67`/`68` | `NOT A` / `NOT B` | inverte todos os bits |
| `70`/`71`/`72` | `CP B` / `CP [addr]` / `CP #value` | `FLAGS <- compara A, operando` |
| `80`/`81`/`82`/`83` | `INC A` / `DEC A` / `INC B` / `DEC B` | incrementa/decrementa registrador |
| `84`/`85` | `INC [addr]` / `DEC [addr]` | incrementa/decrementa direto na memória |
| `90`..`98` | `JP` `JPZ` `JPNZ` `JPC` `JPNC` `JPN` `JNN` `JPV` `JPNV` | desvio condicional para `[addr]` |
| `D0`/`D1` | `CALL [addr]` / `RET` | chama e retorna de uma sub-rotina (1 nível) |
| `A0`..`A7` | `SHL`/`SHR`/`ROL`/`ROR` `A` ou `B` | deslocamentos e rotações |
| `B0` | `INA` | `A <- entrada` (espera confirmação em `INCR`) |
| `B1` | `OUTA` | `OUTPUT <- A` |
| `B2` | `KEY` | `A <- entrada` (leitura direta, sem esperar) |
| `C0` | `RNDA` | `A <- aleatório 0-255` |
| `C1`/`C2` | `DLAYA` / `DLAYB` | `delay(A ou B * 1ms)` |

> **Endereçamento indireto:** `LDA [B]` e `STA [B]` usam o valor de `B`
> como endereço — é o jeito de percorrer um vetor na memória sem
> reescrever o programa (`ldb #$10` / `lda [B]` / `inc b` / `jp loop`).
>
> **Sub-rotinas:** `CALL [addr]` salta guardando o endereço de retorno
> em `$FF`; `RET` volta para lá. Como só existe esse um endereço
> reservado, dá para ter apenas **1 nível** de chamada (uma sub-rotina
> não pode chamar outra sub-rotina).

## Sintaxe do assembler

- Rótulos: `loop:` (podem ficar sozinhos na linha ou seguidos da instrução)
- Endereço/valor em memória: `[endereço]` — ex.: `LDA [$80]`
- Valor imediato: `#valor` — ex.: `LDA #10`
- Números em hexadecimal (`$FF` ou `0xFF`) ou decimal (`255`)
- Comentários com `;`
- Diretivas extras: `ORG endereço` (muda o endereço de montagem) e
  `DB v1,v2,"texto"` (grava bytes/strings diretamente na memória)

Exemplo:

```asm
lda #$00
sta [$80]

loop:
    lda [$80]
    outa
    lda #150
    dlaya
    lda [$80]
    inc a
    sta [$80]
    jp loop
```

## Usando o emulador (`computador-hipotetico-8bits.html`)

Abra o arquivo direto no navegador (funciona offline, em qualquer PC
ou celular, sem instalar nada).

1. Escreva o código no editor (ou escolha um exemplo pronto no menu).
2. Clique em **Montar** (ou `Ctrl+Enter`). Erros de montagem aparecem
   com o número da linha; os bytes gerados ficam visíveis em "Bytes
   montados (hex)".
3. Use **STEP** para executar uma instrução por vez, ou **RUN/STOP**
   para rodar continuamente (a velocidade é ajustável).
4. Quando o programa executa `INA`, ajuste as chaves de dados e aperte
   **INCR** para confirmar a entrada.
5. **ADDR**, **CLR** e **INCR** também servem para programar a memória
   manualmente, byte a byte, como no hardware real.
6. A grade de memória (256 bytes) pode ser editada clicando em
   qualquer célula.

Também é possível salvar/abrir o código-fonte como `.asm`.

## Exemplos incluídos

- **Contador binário** — incrementa e mostra a contagem em `OUTA`.
- **Jogo do reflexo** — sorteia um tempo de espera; o jogador precisa
  apertar o botão assim que os LEDs zerarem.
- **Adivinhe o número** — a CPU sorteia um número de 0 a 15; o jogador
  tenta adivinhar usando as chaves de dados, recebendo dicas de
  maior/menor pelos LEDs de saída.

## Arquivos do projeto

| Arquivo | Descrição |
|---|---|
| `ComputadorHipotetico8bits.ino` | Firmware Arduino: fetch-decode-execute, leitura de botões e controle dos LEDs |
| `Opcodes.txt` | Especificação original do conjunto de instruções |
| `jogo reflexo.txt` | Programa de exemplo (jogo do reflexo) em assembly |
| `computador-hipotetico-8bits.html` | Assembler + emulador standalone |
| `README.md` | Este documento |

## Limitações conhecidas

- A memória é única para dados e programa (256 bytes no total).
- O emulador simplifica o modo de gravação em EEPROM (slots) do
  hardware físico — ele foca em montar/rodar/depurar programas.
