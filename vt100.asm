; ===========================================================================
;  VT-100 / ANSI terminal emulator for ZX Spectrum 48K
;  Software UART 9600 baud 8N1 on the cassette port
;
;      EAR  (port 0FEh, bit 6) = serial RX   <- cassette EAR pin
;      MIC  (port 0FEh, bit 3) = serial TX   -> cassette MIC pin
;      GND                     = common ground
;
;  Screen 32 x 24 characters, ZX ROM font at 0x3C00.
;  CPU 3.546894 MHz (PAL) -> 369.47 T-states per bit at 9600 baud.
;
;  Build (sjasmplus):
;      sjasmplus --raw=terminal.bin vt100.asm
;  or (pasmo):
;      pasmo --bin vt100.asm terminal.bin
;
;  Make a tape image for an emulator (adds the BASIC loader below as a
;  standard ZX tape block, so the tape autostarts):
;      python bin2tap.py terminal.bin          ->  terminal.tap
;
;  Load / start:
;      CLEAR 32767: LOAD "" CODE 32768: RANDOMIZE USR 32768
;  terminal.tap performs exactly that and runs on load.
;
;  Exit to BASIC: CAPS SHIFT + SYMBOL SHIFT + SPACE
; ===========================================================================

ROMFONT     equ 03C00h          ; 96 chars, code 0x20..0x7F, 8 bytes each
ROMPORT     equ 0FEh

COLS        equ 32
ROWS        equ 24

CHRBUF      equ 0C000h          ; 768 bytes of character codes
VTATTR      equ 0C300h          ; 768 bytes of ZX attribute bytes
RXFIFO      equ 0C600h          ; 256 byte receive ring buffer

FIFO_MASK   equ 0FFh

; ---------------------------------------------------------------------------
;  UART timing constants.  Every delay routine below is a
;      LD HL,n / DEC HL / LD A,H / OR L / JR NZ
;  loop = 26*n + 15 T-states including the final RET.
; ---------------------------------------------------------------------------
;  Loop body = 26*N + 15 T including RET, excluding the CALL (17 T).
;  TX intervals are measured OUT-to-OUT and DO include that CALL, hence
;  the odd NOP counts:
;    data bit  = 11(srl..out) + 8 + 17 + body + 13(djnz) + 27 = 76 + body
;                 body = 295  ->  371 T  (ideal 369.5)
;    start bit = 11 + 7(ld b,8) + 17 + body + 27            = 62 + body
;                 body = 307  ->  369 T
RX_PRE_N    equ 14              ; 379 T : pre-roll to the centre of bit 0
RX_BIT_N    equ 10              ; 275 T + 5 NOP = 295 T -> 369 T per bit
RX_BIT_PAD  equ 5
TX_BIT_N    equ 10
TX_BIT_PAD  equ 5               ; 295 T -> data bit = 371 T
TX_START_N  equ 10
TX_START_PAD equ 8              ; 307 T -> start bit = 369 T

BORDER_INIT equ 0               ; border colour while running

            org 32768

; ===========================================================================
;  ENTRY POINT
; ===========================================================================
START:
            di                          ; interrupts off: UART timing is critical
            ld   sp, 32767              ; stack grows down, below the program
            ld   a, (SHADOW_INIT)
            ld   (SHADOW), a
            out  (ROMPORT), a           ; idle MIC high, border set

            call FIFO_RESET
            call KEYBUF_RESET
            call VT_RESET               ; clear buffers + full redraw

; ---------------------------------------------------------------------------
;  Main loop
; ---------------------------------------------------------------------------
MAIN:
            ld   a, (SHADOW)            ; keep the port in the idle state
            out  (ROMPORT), a           ; (also refreshes border after TX)

            in   a, (ROMPORT)           ; --- tight RX poll 1 ---
            add  a, a                   ; bit7 <- EAR
            call p, RX_START            ; P set = EAR low = start bit
            call FIFO_GET
            jr   c, DO_CHAR

            in   a, (ROMPORT)           ; --- tight RX poll 2 ---
            add  a, a
            call p, RX_START
            call FIFO_GET
            jr   c, DO_CHAR

            call KEY_SCAN               ; polls RX internally every row
            jr   MAIN

DO_CHAR:
            call VT_PUTCHAR
            jr   MAIN

; ===========================================================================
;  SOFTWARE UART - RECEIVER
; ===========================================================================

; ---------------------------------------------------------------------------
;  RX_START - called (via CALL P) the moment a start bit is seen.
;  Receives one byte and pushes it into the FIFO.
;  Preserves BC, DE, HL. Clobbers AF.
; ---------------------------------------------------------------------------
RX_START:
            push hl
            push de
            push bc
            call RX_BYTE
            call FIFO_PUT
            pop  bc
            pop  de
            pop  hl
            ret

; ---------------------------------------------------------------------------
;  RX_BYTE - blocking receive of one 8N1 byte. Returns A = byte.
;
;  Bit 0 is sampled 1.5 bit periods after the true falling edge. The
;  detection latency L is bounded by the longest gap between two polls
;  (<= ~150 T here); the preamble is tuned for the average, so the sample
;  lands between roughly 1.3 and 1.7 bit times - well inside the bit.
;
;  Data is shifted with RR D, so the LSB-first line order ends up as the
;  natural byte order (no bit reversal needed).
; ---------------------------------------------------------------------------
RX_BYTE:
            ld   d, 0
            ld   b, 8
            call RX_PRE_DELAY

RX_BIT:
            in   a, (ROMPORT)           ; 11
            and  40h                    ;  7   isolate EAR
            add  a, a                   ;  4
            add  a, a                   ;  4   EAR -> carry
            rr   d                      ;  8   shift into the byte
            dec  b                      ;  4
            jr   z, RX_DONE             ;  7/12
            call RX_BIT_DELAY           ; 17 + 295
            jr   RX_BIT                 ; 12

RX_DONE:
            ld   a, d
            ret

; 379 T: pre-roll to the centre of bit 0 (see header comment)
RX_PRE_DELAY:
            ld   hl, RX_PRE_N
RXPD1:
            dec  hl
            ld   a, h
            or   l
            jr   nz, RXPD1
            ret

; 295 T: exactly one bit period minus the sampling code
RX_BIT_DELAY:
            ld   hl, RX_BIT_N
RXBD1:
            dec  hl
            ld   a, h
            or   l
            jr   nz, RXBD1
            nop
            nop
            nop
            nop
            nop
            ret

; ===========================================================================
;  SOFTWARE UART - TRANSMITTER
; ===========================================================================

; ---------------------------------------------------------------------------
;  TX_CHAR - A = byte to send. Blocks for ~1.04 ms at 9600 baud.
;  D is the port shadow with the MIC bit already cleared.
; ---------------------------------------------------------------------------
TX_CHAR:
            push af
            ld   a, (SHADOW)
            and  0F7h                   ; MIC low for the start bit
            ld   d, a
            pop  af
            jp   TX_BYTE

; ---------------------------------------------------------------------------
;  TX_BYTE - A = byte. Sends start bit, 8 data bits LSB first, stop bit.
;  Clobbers AF, B, D, E, HL.
; ---------------------------------------------------------------------------
TX_BYTE:
            ld   e, a

            ld   a, d                   ; ---- start bit (low) ----
            out  (ROMPORT), a
            ld   b, 8
            call TX_START_DELAY

TX_LOOP:
            ld   a, e                   ; ---- data bit ----
            and  1
            add  a, a
            add  a, a
            add  a, a                   ; bit -> MIC position
            or   d
            out  (ROMPORT), a
            srl  e
            call TX_BIT_DELAY
            djnz TX_LOOP

            ld   a, d                   ; ---- stop bit (high) ----
            or   8
            out  (ROMPORT), a
            ret

; 307 T body -> 369 T start bit (out .. out)
TX_START_DELAY:
            ld   hl, TX_START_N
TXSD1:
            dec  hl
            ld   a, h
            or   l
            jr   nz, TXSD1
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            nop
            ret

; 295 T body -> 371 T data bit (out .. out)
TX_BIT_DELAY:
            ld   hl, TX_BIT_N
TXBD2:
            dec  hl
            ld   a, h
            or   l
            jr   nz, TXBD2
            nop
            nop
            nop
            nop
            nop
            ret

; ===========================================================================
;  RECEIVE FIFO (256 byte ring, head/tail are byte indices into page C6)
; ===========================================================================

FIFO_RESET:
            xor  a
            ld   (FIFO_HEAD), a
            ld   (FIFO_TAIL), a
            ret

; A = byte. Preserves everything except AF.
FIFO_PUT:
            push af
            push hl
            push de
            ld   e, a
            ld   a, (FIFO_HEAD)
            ld   h, RXFIFO >> 8
            ld   l, a
            ld   (hl), e
            inc  a
            ld   (FIFO_HEAD), a
            pop  de
            pop  hl
            pop  af
            ret

; Carry set + A = byte, or carry clear when empty.
FIFO_GET:
            push hl
            push bc
            ld   a, (FIFO_TAIL)
            ld   b, a
            ld   a, (FIFO_HEAD)
            cp   b
            jr   z, FIFO_EMPTY
            ld   h, RXFIFO >> 8
            ld   l, b
            ld   a, (hl)
            ld   c, a                   ; keep the byte
            inc  b
            ld   a, b
            ld   (FIFO_TAIL), a
            ld   a, c                   ; restore the byte
            pop  bc
            pop  hl
            scf
            ret
FIFO_EMPTY:
            pop  bc
            pop  hl
            or   a                      ; clear carry
            ret

; ===========================================================================
;  SCREEN / CELL PRIMITIVES
; ===========================================================================

; ---------------------------------------------------------------------------
;  CALCIDX - B = row, C = col  ->  L = (row&7)*32 + col , A = row>>3
;
;  The ZX display file address of a cell is
;      H = 0x40 | (row & 0x18) | pixel_row ,  L = (row & 7) * 32 + col
;  so a whole cell is 8 consecutive INC H steps - no 16 bit arithmetic.
;  A comes back as the page offset for CHRBUF / VTATTR / attributes.
; ---------------------------------------------------------------------------
CALCIDX:
            ld   a, b
            and  7
            rrca
            rrca
            rrca                        ; (row & 7) * 32
            or   c
            ld   l, a
            ld   a, b
            rrca
            rrca
            rrca                        ; (row>>3) | ((row&7)<<5)
            and  3
            ret                         ; C, B, L preserved apart from L

; ---------------------------------------------------------------------------
;  DRAWCELL - B = row, C = col, A = ZX attribute. Draws the cell whose
;  character code lives in CHRBUF.  Polls the UART between byte pairs so a
;  full screen redraw never loses incoming data.
; ---------------------------------------------------------------------------
DRAWCELL:
            ld   (DRWATTR), a
            call CALCIDX                ; L = idx, A = row>>3

            ld   (DRWROFF), a
            add  a, VTATTR >> 8         ; attribute address -> DRWAPTR
            ld   h, a
            ld   (DRWAPTR), hl

            ld   a, b                   ; screen base -> DRWSPTR
            and  18h
            or   40h
            ld   h, a
            ld   (DRWSPTR), hl          ; L is still idx

            ld   a, (DRWROFF)           ; fetch the character code
            add  a, CHRBUF >> 8
            ld   h, a
            ld   a, (hl)

            cp   20h                    ; font lookup
            jr   c, DC_SPACE
            cp   7Fh
            jr   nc, DC_SPACE
            sub  20h
            add  a, a
            add  a, a
            add  a, a                   ; (code-0x20)*8
            ld   l, a
            ld   h, 0
            ld   de, ROMFONT
            add  hl, de
            ex   de, hl
            jr   DC_FONT_OK
DC_SPACE:
            ld   de, ROMFONT            ; char 0x20 is 8 zero bytes
DC_FONT_OK:
            ld   hl, (DRWSPTR)
            ld   b, 8
DC_LOOP:
            ld   a, (de)                ; --- byte pair, then poll ---
            ld   (hl), a
            inc  de
            inc  h
            ld   a, (de)
            ld   (hl), a
            inc  de
            inc  h
            in   a, (ROMPORT)
            add  a, a
            call p, RX_START
            djnz DC_LOOP

            ld   hl, (DRWAPTR)
            ld   a, (DRWATTR)
            ld   (hl), a
            ret

; ---------------------------------------------------------------------------
;  ERASE_CELL - B = row, C = col. Fills the cell with a space using the
;  current SGR attributes and redraws it.
; ---------------------------------------------------------------------------
ERASE_CELL:
            push bc
            call CALCIDX
            ld   h, CHRBUF >> 8
            add  a, h
            ld   h, a
            ld   (hl), 20h
            pop  bc
            push bc
            call CALCIDX
            ld   h, VTATTR >> 8
            add  a, h
            ld   h, a
            call COMPOSE_ATTR
            ld   (hl), a
            pop  bc
            call CELLATTR
            jp   DRAWCELL

; ---------------------------------------------------------------------------
;  CELLATTR - B = row, C = col -> A = stored ZX attribute
; ---------------------------------------------------------------------------
CELLATTR:
            call CALCIDX
            add  a, VTATTR >> 8
            ld   h, a
            ld   a, (hl)
            ret

; ---------------------------------------------------------------------------
;  COMPOSE_ATTR - A = ZX attribute built from CUR_PAPER / CUR_INK / CUR_ATTR
; ---------------------------------------------------------------------------
COMPOSE_ATTR:
            ld   a, (CUR_PAPER)
            add  a, a
            add  a, a
            add  a, a                   ; paper << 3
            ld   c, a
            ld   a, (CUR_INK)
            or   c
            ld   b, a

            ld   a, (CUR_ATTR)
            bit  1, a                   ; reverse video -> swap ink/paper
            jr   z, CA_NOREV
            ld   a, (CUR_INK)
            add  a, a
            add  a, a
            add  a, a
            ld   b, a
            ld   a, (CUR_PAPER)
            or   b
            ld   b, a
CA_NOREV:
            ld   a, (CUR_ATTR)
            bit  0, a                   ; bold -> BRIGHT
            jr   z, CA_NOBOLD
            set  6, b
CA_NOBOLD:
            ld   a, (CUR_ATTR)
            bit  3, a                   ; blink -> FLASH
            jr   z, CA_NOFLASH
            set  7, b
CA_NOFLASH:
            ld   a, b
            ret

; ---------------------------------------------------------------------------
;  REDRAW_ALL - repaint every cell from CHRBUF / VTATTR
; ---------------------------------------------------------------------------
REDRAW_ALL:
            ld   a, 0FFh
            ld   (OLDROW), a            ; no stale cursor to erase
            ld   b, 0
RA_ROW:
            ld   c, 0
RA_COL:
            push bc
            call CELLATTR
            call DRAWCELL
            pop  bc
            inc  c
            ld   a, c
            cp   COLS
            jr   c, RA_COL
            inc  b
            ld   a, b
            cp   ROWS
            jr   c, RA_ROW
            jp   CUR_UPDATE

; ===========================================================================
;  CURSOR
; ===========================================================================

; Erase the old cursor cell, draw the cursor at CUR_ROW/CUR_COL.
CUR_UPDATE:
            ld   a, (OLDROW)
            cp   0FFh
            jr   z, CU_NOCLEAR
            ld   b, a
            ld   a, (OLDCOL)
            ld   c, a
            call CELLATTR
            call DRAWCELL
CU_NOCLEAR:
            ld   a, (CUR_ROW)
            ld   b, a
            ld   a, (CUR_COL)
            ld   c, a
            call CELLATTR
            xor  3Fh                    ; invert ink and paper -> block cursor
            call DRAWCELL
            ld   a, (CUR_ROW)
            ld   (OLDROW), a
            ld   a, (CUR_COL)
            ld   (OLDCOL), a
            ret

; ===========================================================================
;  VT-100 / ANSI PARSER
; ===========================================================================

; ---------------------------------------------------------------------------
;  VT_PUTCHAR - A = character from the host
; ---------------------------------------------------------------------------
VT_PUTCHAR:
            ld   (PCH), a
            ld   a, (PSTATE)
            or   a
            jr   z, VT_GROUND
            dec  a
            jr   z, VT_ESC
            dec  a
            jp   z, VT_CSI
            dec  a
            jp   z, VT_SKIP1
            ; PSTATE 4 = OSC, swallow until BEL
            ld   a, (PCH)
            cp   07h
            ret  nz
            xor  a
            ld   (PSTATE), a
            ret

VT_GROUND:
            ld   a, (PCH)
            cp   1Bh                    ; ESC
            jr   z, VG_ESC
            cp   0Ah                    ; LF
            jp   z, VT_LF
            cp   0Dh                    ; CR
            jp   z, VT_CR
            cp   08h                    ; BS
            jp   z, VT_BS
            cp   09h                    ; HT
            jp   z, VT_TAB
            cp   07h                    ; BEL
            ret  z
            cp   20h
            ret  c                      ; other C0 controls ignored
            jp   VT_PUTGLYPH
VG_ESC:
            ld   a, 1
            ld   (PSTATE), a
            ret

VT_ESC:
            ld   a, (PCH)
            cp   5Bh                    ; '['  -> CSI
            jr   z, VE_CSI
            cp   5Dh                    ; ']'  -> OSC
            jr   z, VE_OSC
            cp   63h                    ; 'c'  -> RIS (reset)
            jp   z, VT_RESET
            cp   37h                    ; '7'  -> DECSC save cursor
            jp   z, VT_SAVECUR
            cp   38h                    ; '8'  -> DECRC restore cursor
            jp   z, VT_RESTCUR
            cp   44h                    ; 'D'  -> IND
            jp   z, VT_LF
            cp   4Dh                    ; 'M'  -> RI (reverse index)
            jp   z, VT_RI
            cp   45h                    ; 'E'  -> NEL
            jp   z, VT_NEL
            cp   28h                    ; '('
            jr   z, VE_SKIP
            cp   29h                    ; ')'
            jr   z, VE_SKIP
            cp   23h                    ; '#'
            jr   z, VE_SKIP
VE_GROUND:
            xor  a
            ld   (PSTATE), a
            ret
VE_CSI:
            ld   a, 2
            ld   (PSTATE), a
            xor  a
            ld   (PNUM), a
            ld   (PCUR), a
            ld   (PINNUM), a
            ld   (PPRIV), a
            ret
VE_OSC:
            ld   a, 4
            ld   (PSTATE), a
            ret
VE_SKIP:
            ld   a, 3
            ld   (PSTATE), a
            ret

VT_SKIP1:                           ; swallow exactly one character
            xor  a
            ld   (PSTATE), a
            ret

; ---------------------------------------------------------------------------
;  CSI parameter collection
; ---------------------------------------------------------------------------
VT_CSI:
            ld   a, (PCH)
            cp   30h
            jr   c, CI_NONUM
            cp   3Ah
            jr   nc, CI_NONUM
            sub  30h                    ; --- digit ---
            ld   b, a
            ld   a, (PCUR)
            ld   c, a
            add  a, a
            add  a, a
            add  a, c                   ; *5
            add  a, a                   ; *10
            add  a, b
            ld   (PCUR), a
            ld   a, 1
            ld   (PINNUM), a
            ret
CI_NONUM:
            cp   3Bh                    ; ';'
            jr   z, CI_SEMI
            cp   3Fh                    ; '?'
            jr   z, CI_PRIV
            cp   3Eh                    ; '>'
            jr   z, CI_PRIV
            jp   VT_CSI_FINAL
CI_SEMI:
            call VT_PUSH_PARAM
            ret
CI_PRIV:
            ld   a, 1
            ld   (PPRIV), a
            ret

VT_PUSH_PARAM:
            ld   a, (PNUM)
            cp   8
            ret  nc
            ld   e, a
            ld   d, 0
            ld   hl, PARAM
            add  hl, de
            ld   a, (PCUR)
            ld   (hl), a
            inc  e
            ld   a, e
            ld   (PNUM), a
            xor  a
            ld   (PCUR), a
            ld   (PINNUM), a
            ret

VT_CSI_FINAL:
            ld   a, (PINNUM)
            or   a
            call nz, VT_PUSH_PARAM
            xor  a
            ld   (PSTATE), a
            ld   a, (PPRIV)
            or   a
            ret  nz                     ; DEC private sequences ignored
            ld   a, (PCH)
            cp   41h
            jp   z, CSI_CUU
            cp   42h
            jp   z, CSI_CUD
            cp   43h
            jp   z, CSI_CUF
            cp   44h
            jp   z, CSI_CUB
            cp   45h
            jp   z, CSI_CNL
            cp   46h
            jp   z, CSI_CPL
            cp   47h
            jp   z, CSI_CHA
            cp   48h
            jp   z, CSI_CUP
            cp   66h
            jp   z, CSI_CUP
            cp   64h
            jp   z, CSI_VPA
            cp   4Ah
            jp   z, CSI_ED
            cp   4Bh
            jp   z, CSI_EL
            cp   6Dh
            jp   z, CSI_SGR
            cp   73h
            jp   z, VT_SAVECUR
            cp   75h
            jp   z, VT_RESTCUR
            ret

; --- parameter helpers ----------------------------------------------------
GETP0:                              ; PARAM[0] or 0
            ld   a, (PNUM)
            or   a
            jr   z, GP0_Z
            ld   a, (PARAM)
            ret
GP0_Z:
            xor  a
            ret

GETP1:                              ; PARAM[0], missing/0 -> 1
            call GETP0
            or   a
            ret  nz
            inc  a
            ret

GETP2:                              ; PARAM[1], missing/0 -> 1
            ld   a, (PNUM)
            cp   2
            jr   c, GP2_1
            ld   a, (PARAM+1)
            or   a
            ret  nz
GP2_1:
            ld   a, 1
            ret

; --- cursor movement ------------------------------------------------------
CSI_CUU:
            call GETP1
            ld   b, a
            ld   a, (CUR_ROW)
            sub  b
            jr   nc, CUU_OK
            xor  a
CUU_OK:
            ld   (CUR_ROW), a
            jp   CUR_UPDATE

CSI_CUD:
            call GETP1
            ld   b, a
            ld   a, (CUR_ROW)
            add  a, b
            cp   ROWS
            jr   c, CUD_OK
            ld   a, ROWS-1
CUD_OK:
            ld   (CUR_ROW), a
            jp   CUR_UPDATE

CSI_CUF:
            call GETP1
            ld   b, a
            ld   a, (CUR_COL)
            add  a, b
            cp   COLS
            jr   c, CUF_OK
            ld   a, COLS-1
CUF_OK:
            ld   (CUR_COL), a
            jp   CUR_UPDATE

CSI_CUB:
            call GETP1
            ld   b, a
            ld   a, (CUR_COL)
            sub  b
            jr   nc, CUB_OK
            xor  a
CUB_OK:
            ld   (CUR_COL), a
            jp   CUR_UPDATE

CSI_CNL:                            ; down n, column 0
            xor  a
            ld   (CUR_COL), a
            jp   CSI_CUD

CSI_CPL:                            ; up n, column 0
            xor  a
            ld   (CUR_COL), a
            jp   CSI_CUU

CSI_CHA:
            call GETP1
            dec  a
            cp   COLS
            jr   c, CHA_OK
            ld   a, COLS-1
CHA_OK:
            ld   (CUR_COL), a
            jp   CUR_UPDATE

CSI_VPA:
            call GETP1
            dec  a
            cp   ROWS
            jr   c, VPA_OK
            ld   a, ROWS-1
VPA_OK:
            ld   (CUR_ROW), a
            jp   CUR_UPDATE

CSI_CUP:
            call GETP1
            dec  a
            cp   ROWS
            jr   c, CUP_R
            ld   a, ROWS-1
CUP_R:
            ld   (CUR_ROW), a
            call GETP2
            dec  a
            cp   COLS
            jr   c, CUP_C
            ld   a, COLS-1
CUP_C:
            ld   (CUR_COL), a
            jp   CUR_UPDATE

VT_SAVECUR:
            ld   a, (CUR_ROW)
            ld   (SAV_ROW), a
            ld   a, (CUR_COL)
            ld   (SAV_COL), a
            ret

VT_RESTCUR:
            ld   a, (SAV_ROW)
            ld   (CUR_ROW), a
            ld   a, (SAV_COL)
            ld   (CUR_COL), a
            jp   CUR_UPDATE

; --- erase ----------------------------------------------------------------
CSI_EL:
            call GETP0
            cp   2
            jr   z, EL_ALL
            cp   1
            jr   z, EL_LEFT
            ld   a, (CUR_COL)
            ld   (CLR1), a
            ld   a, COLS-1
            ld   (CLR2), a
            jr   EL_GO
EL_LEFT:
            xor  a
            ld   (CLR1), a
            ld   a, (CUR_COL)
            ld   (CLR2), a
            jr   EL_GO
EL_ALL:
            xor  a
            ld   (CLR1), a
            ld   a, COLS-1
            ld   (CLR2), a
EL_GO:
            ld   a, (CUR_ROW)
            ld   (CLRROW), a
            call ERASE_LINE
            jp   CUR_UPDATE

CSI_ED:
            call GETP0
            cp   2
            jr   z, ED_ALL
            cp   1
            jr   z, ED_ABOVE
            ; --- 0: cursor .. end of screen ---
            ld   a, (CUR_ROW)
            ld   (CLRROW), a
            ld   a, (CUR_COL)
            ld   (CLR1), a
            ld   a, COLS-1
            ld   (CLR2), a
            call ERASE_LINE
            ld   a, (CUR_ROW)
            inc  a
            cp   ROWS
            jr   nc, ED_DONE
            ld   (CLRROW), a
            ld   a, ROWS-1
            ld   (CLRLAST), a
            call CLEAR_ROWS
            jr   ED_DONE
ED_ABOVE:
            ld   a, (CUR_ROW)
            or   a
            jr   z, ED_TOPLINE
            dec  a
            ld   (CLRLAST), a
            xor  a
            ld   (CLRROW), a
            call CLEAR_ROWS
ED_TOPLINE:
            ld   a, (CUR_ROW)
            ld   (CLRROW), a
            xor  a
            ld   (CLR1), a
            ld   a, (CUR_COL)
            ld   (CLR2), a
            call ERASE_LINE
            jr   ED_DONE
ED_ALL:
            xor  a
            ld   (CLRROW), a
            ld   a, ROWS-1
            ld   (CLRLAST), a
            call CLEAR_ROWS
ED_DONE:
            jp   CUR_UPDATE

; ERASE_LINE - clear CLR1..CLR2 of row CLRROW
ERASE_LINE:
            ld   a, (CLRROW)
            ld   b, a
            ld   a, (CLR1)
            ld   c, a
            call ERASE_CELL
            ld   a, (CLR2)
            ld   b, a
            ld   a, (CLR1)
            cp   b
            ret  z
            inc  a
            ld   (CLR1), a
            jr   ERASE_LINE

; CLEAR_ROWS - clear whole rows CLRROW..CLRLAST
CLEAR_ROWS:
            ld   a, (CLRROW)
            ld   b, a
            ld   a, (CLRLAST)
            cp   b
            ret  c
            xor  a
            ld   (CLR1), a
            ld   a, COLS-1
            ld   (CLR2), a
            call ERASE_LINE
            ld   a, (CLRROW)
            inc  a
            ld   (CLRROW), a
            jr   CLEAR_ROWS

; --- SGR ------------------------------------------------------------------
ANSI2ZX:
            defb 0, 2, 4, 6, 1, 3, 5, 7      ; ANSI 30..37 -> ZX ink/paper

CSI_SGR:
            ld   a, (PNUM)
            or   a
            jr   nz, SGR_LOOP
            xor  a
            ld   (CUR_ATTR), a
            ld   a, 7
            ld   (CUR_INK), a
            xor  a
            ld   (CUR_PAPER), a
            jp   CUR_UPDATE
SGR_LOOP:
            ld   b, a
            ld   hl, PARAM
SGR_NEXT:
            ld   a, (hl)
            inc  hl
            push hl
            push bc
            call SGR_ONE
            pop  bc
            pop  hl
            djnz SGR_NEXT
            jp   CUR_UPDATE

SGR_ONE:
            or   a
            jr   z, SGR_RESET
            cp   1
            jr   z, SGR_BOLD
            cp   4
            jr   z, SGR_UL
            cp   5
            jr   z, SGR_BLINK
            cp   7
            jr   z, SGR_REV
            cp   22
            jr   z, SGR_NBOLD
            cp   24
            jr   z, SGR_NUL
            cp   25
            jr   z, SGR_NBLINK
            cp   27
            jr   z, SGR_NREV
            cp   30
            jr   c, SGR_END
            cp   38
            jr   nc, SGR_BG
            sub  30                     ; foreground colour
            ld   e, a
            ld   d, 0
            ld   hl, ANSI2ZX
            add  hl, de
            ld   a, (hl)
            ld   (CUR_INK), a
            ret
SGR_BG:
            cp   40
            jr   c, SGR_END
            cp   48
            jr   nc, SGR_END
            sub  40                     ; background colour
            ld   e, a
            ld   d, 0
            ld   hl, ANSI2ZX
            add  hl, de
            ld   a, (hl)
            ld   (CUR_PAPER), a
            ret
SGR_RESET:
            xor  a
            ld   (CUR_ATTR), a
            ld   a, 7
            ld   (CUR_INK), a
            xor  a
            ld   (CUR_PAPER), a
            ret
SGR_BOLD:
            ld   a, (CUR_ATTR)
            or   1
            ld   (CUR_ATTR), a
            ret
SGR_UL:
            ld   a, (CUR_ATTR)
            or   4
            ld   (CUR_ATTR), a
            ret
SGR_BLINK:
            ld   a, (CUR_ATTR)
            or   8
            ld   (CUR_ATTR), a
            ret
SGR_REV:
            ld   a, (CUR_ATTR)
            or   2
            ld   (CUR_ATTR), a
            ret
SGR_NBOLD:
            ld   a, (CUR_ATTR)
            and  0FEh
            ld   (CUR_ATTR), a
            ret
SGR_NUL:
            ld   a, (CUR_ATTR)
            and  0FBh
            ld   (CUR_ATTR), a
            ret
SGR_NBLINK:
            ld   a, (CUR_ATTR)
            and  0F7h
            ld   (CUR_ATTR), a
            ret
SGR_NREV:
            ld   a, (CUR_ATTR)
            and  0FDh
            ld   (CUR_ATTR), a
            ret
SGR_END:
            ret

; ===========================================================================
;  CHARACTER OUTPUT / SCROLLING
; ===========================================================================

; ---------------------------------------------------------------------------
;  VT_PUTGLYPH - A = printable character
; ---------------------------------------------------------------------------
VT_PUTGLYPH:
            ld   (PCH), a

            ld   a, (OLDROW)            ; 1. erase the old cursor
            cp   0FFh
            jr   z, PG_NOCLEAR
            ld   b, a
            ld   a, (OLDCOL)
            ld   c, a
            call CELLATTR
            call DRAWCELL
PG_NOCLEAR:
            ld   a, (CUR_ROW)           ; 2. store char + attribute
            ld   b, a
            ld   a, (CUR_COL)
            ld   c, a
            call CALCIDX                ; L = idx, A = row>>3
            ld   (DRWROFF), a
            ld   h, CHRBUF >> 8
            add  a, h
            ld   h, a
            ld   a, (PCH)
            ld   (hl), a
            ld   a, (DRWROFF)
            ld   h, VTATTR >> 8
            add  a, h
            ld   h, a
            call COMPOSE_ATTR
            ld   (hl), a

            ld   a, (CUR_ROW)           ; 3. draw the cell
            ld   b, a
            ld   a, (CUR_COL)
            ld   c, a
            call CELLATTR
            call DRAWCELL

            call VT_ADVANCE             ; 4. move on
            jp   CUR_UPDATE             ; 5. draw the new cursor

VT_ADVANCE:
            ld   a, (CUR_COL)
            inc  a
            cp   COLS
            jr   c, VA_SET
            xor  a
            ld   (CUR_COL), a
            jp   VT_LF
VA_SET:
            ld   (CUR_COL), a
            ret

; ---------------------------------------------------------------------------
;  VT_LF - line feed, scrolls when past the last row
; ---------------------------------------------------------------------------
VT_LF:
            ld   a, (CUR_ROW)
            inc  a
            cp   ROWS
            jr   c, LF_OK
            call VT_SCROLL
            ld   a, ROWS-1
LF_OK:
            ld   (CUR_ROW), a
            ret

VT_NEL:
            xor  a
            ld   (CUR_COL), a
            jp   VT_LF

VT_RI:                              ; reverse index (no scroll region support)
            ld   a, (CUR_ROW)
            or   a
            ret  z
            dec  a
            ld   (CUR_ROW), a
            jp   CUR_UPDATE

VT_CR:
            xor  a
            ld   (CUR_COL), a
            ret

VT_BS:
            ld   a, (CUR_COL)
            or   a
            ret  z
            dec  a
            ld   (CUR_COL), a
            ret

VT_TAB:
            ld   a, (CUR_COL)
            or   7
            inc  a
            cp   COLS
            jr   c, TAB_OK
            ld   a, COLS-1
TAB_OK:
            ld   (CUR_COL), a
            ret

; ---------------------------------------------------------------------------
;  VT_SCROLL - move rows 1..23 up by one, blank the last row, full repaint
; ---------------------------------------------------------------------------
VT_SCROLL:
            ld   hl, CHRBUF+COLS
            ld   de, CHRBUF
            ld   bc, COLS*(ROWS-1)
            ldir
            ld   hl, CHRBUF+COLS*(ROWS-1)
            ld   de, CHRBUF+COLS*(ROWS-1)+1
            ld   bc, COLS-1
            ld   (hl), 20h
            ldir

            ld   hl, VTATTR+COLS
            ld   de, VTATTR
            ld   bc, COLS*(ROWS-1)
            ldir
            ld   hl, VTATTR+COLS*(ROWS-1)
            ld   de, VTATTR+COLS*(ROWS-1)+1
            ld   bc, COLS-1
            ld   (hl), 07h
            ldir

            jp   REDRAW_ALL

; ---------------------------------------------------------------------------
;  VT_RESET - RIS / power on
; ---------------------------------------------------------------------------
VT_RESET:
            xor  a
            ld   (CUR_ROW), a
            ld   (CUR_COL), a
            ld   (SAV_ROW), a
            ld   (SAV_COL), a
            ld   (PSTATE), a
            ld   (CUR_ATTR), a
            ld   (CUR_PAPER), a
            ld   (PPRIV), a
            ld   (PNUM), a
            ld   (PINNUM), a
            ld   (PCUR), a
            ld   a, 7
            ld   (CUR_INK), a

            ld   hl, CHRBUF              ; blank the character buffer
            ld   de, CHRBUF+1
            ld   bc, COLS*ROWS-1
            ld   (hl), 20h
            ldir
            ld   hl, VTATTR              ; default attribute
            ld   de, VTATTR+1
            ld   bc, COLS*ROWS-1
            ld   (hl), 07h
            ldir

            jp   REDRAW_ALL

; ===========================================================================
;  KEYBOARD
; ===========================================================================

KEYBUF_RESET:
            ld   hl, KEYBUF
            ld   b, 8
            xor  a
KBR1:
            ld   (hl), a
            inc  hl
            djnz KBR1
            ret

; ---------------------------------------------------------------------------
;  KEY_SCAN - reads the eight half rows, detects newly pressed keys and
;  transmits the corresponding ASCII / ANSI sequence.  One UART poll is
;  inserted per half row so the receiver never goes blind for long.
; ---------------------------------------------------------------------------
KEY_SCAN:
            ld   hl, KEYBUF
            ld   ix, KEYROWS
            ld   c, ROMPORT
            ld   d, 8

KS_LOOP:
            ld   b, (ix)                ; B = port high byte
            inc  ix
            in   a, (c)
            cpl                         ; 1 = pressed
            and  1Fh
            ld   e, a                   ; new state
            ld   a, (hl)                ; old state
            xor  e
            and  e                      ; newly pressed = changed AND new
            ld   (hl), e
            or   a
            jr   z, KS_NOKEY
            push hl
            push de
            ld   a, l
            sub  KEYBUF & 0FFh          ; half row index 0..7
            ld   b, a
            call KEY_HANDLE             ; A = bit mask, B = row
            pop  de
            pop  hl
KS_NOKEY:
            inc  hl
            in   a, (ROMPORT)           ; UART poll
            add  a, a
            call p, RX_START
            dec  d
            jr   nz, KS_LOOP

            ld   a, (KEYBUF)            ; CAPS SHIFT + SYMBOL SHIFT + SPACE
            and  1                      ;   -> back to BASIC
            ret  z
            ld   a, (KEYBUF+7)
            and  3
            cp   3
            ret  nz
            ld   a, (SHADOW)
            out  (ROMPORT), a
            ei
            jp   0

; ---------------------------------------------------------------------------
;  KEY_HANDLE - A = newly pressed bit mask, B = half row 0..7
; ---------------------------------------------------------------------------
KEY_HANDLE:
            ld   c, 0                   ; C = bit index of the lowest set bit
KH_SHIFT:
            rrca
            jr   c, KH_GOT
            inc  c
            jr   KH_SHIFT
KH_GOT:
            ld   a, b                   ; key index = row*5 + bit
            add  a, a
            add  a, a
            add  a, b
            add  a, c
            ld   c, a                   ; C = key index 0..39

            ld   a, (KEYBUF)            ; --- CAPS SHIFT held? ---
            and  1
            jr   z, KH_NOCAPS

            ld   a, c                   ; cursor keys
            cp   19                     ; '5'
            jr   z, KH_LEFT
            cp   24                     ; '6'
            jr   z, KH_DOWN
            cp   23                     ; '7'
            jr   z, KH_UP
            cp   22                     ; '8'
            jr   z, KH_RIGHT
            cp   20                     ; '0' = DELETE
            jr   z, KH_BS

            ld   hl, KEYBASE            ; letters -> upper case
            ld   e, c
            ld   d, 0
            add  hl, de
            ld   a, (hl)
            cp   61h
            ret  c
            cp   7Bh
            ret  nc
            sub  20h
            jp   TX_CHAR

KH_NOCAPS:
            ld   a, (KEYBUF+7)          ; --- SYMBOL SHIFT held? ---
            and  2
            jr   z, KH_PLAIN
            ld   hl, KEYSYM
            jr   KH_LOOK
KH_PLAIN:
            ld   hl, KEYBASE
KH_LOOK:
            ld   e, c
            ld   d, 0
            add  hl, de
            ld   a, (hl)
            or   a
            ret  z                      ; unmapped key
            jp   TX_CHAR

KH_LEFT:
            ld   hl, SEQ_LEFT
            jr   KH_SEQ
KH_DOWN:
            ld   hl, SEQ_DOWN
            jr   KH_SEQ
KH_UP:
            ld   hl, SEQ_UP
            jr   KH_SEQ
KH_RIGHT:
            ld   hl, SEQ_RIGHT
            jr   KH_SEQ
KH_BS:
            ld   a, 08h
            jp   TX_CHAR
KH_SEQ:
            ld   a, (hl)
            call TX_CHAR
            inc  hl
            ld   a, (hl)
            call TX_CHAR
            inc  hl
            ld   a, (hl)
            jp   TX_CHAR

; ---------------------------------------------------------------------------
;  Key tables.  Index = row*5 + bit, matching KEYROWS order:
;    row0 CAPS  Z  X  C  V      row4  0  9  8  7  6
;    row1  A  S  D  F  G        row5  P  O  I  U  Y
;    row2  Q  W  E  R  T        row6 ENT L  K  J  H
;    row3  1  2  3  4  5        row7 SPACE SS M  N  B
; ---------------------------------------------------------------------------
KEYROWS:
            defb 0FEh, 0FDh, 0FBh, 0F7h, 0EFh, 0DFh, 0BFh, 07Fh

KEYBASE:
            defb 0,     'z', 'x', 'c', 'v'
            defb 'a',   's', 'd', 'f', 'g'
            defb 'q',   'w', 'e', 'r', 't'
            defb '1',   '2', '3', '4', '5'
            defb '0',   '9', '8', '7', '6'
            defb 'p',   'o', 'i', 'u', 'y'
            defb 0Dh,   'l', 'k', 'j', 'h'
            defb 20h,   0,   'm', 'n', 'b'

KEYSYM:
            defb 0,     ':', '#', '?', '/'
            defb '~',   '|', 5Ch, '{', '}'
            defb 0,     0,   0,   '<', '>'
            defb '!',   '@', '#', '$', '%'
            defb '_',   ')', '(', 27h, '&'
            defb '"',   ';', 0,   0,   0
            defb 0,     '=', '+', '-', '^'
            defb 0,     0,   '.', ',', '*'

SEQ_UP:
            defb 1Bh, 5Bh, 41h
SEQ_DOWN:
            defb 1Bh, 5Bh, 42h
SEQ_RIGHT:
            defb 1Bh, 5Bh, 43h
SEQ_LEFT:
            defb 1Bh, 5Bh, 44h

SHADOW_INIT:
            defb (BORDER_INIT << 4) | 08h

; ===========================================================================
;  VARIABLES
; ===========================================================================
CUR_ROW:    defb 0
CUR_COL:    defb 0
SAV_ROW:    defb 0
SAV_COL:    defb 0
OLDROW:     defb 0FFh
OLDCOL:     defb 0
CUR_ATTR:   defb 0
CUR_INK:    defb 7
CUR_PAPER:  defb 0

PSTATE:     defb 0
PNUM:       defb 0
PCUR:       defb 0
PINNUM:     defb 0
PPRIV:      defb 0
PARAM:      defs 8
PCH:        defb 0

DRWATTR:    defb 0
DRWROFF:    defb 0
DRWAPTR:    defw 0
DRWSPTR:    defw 0

CLRROW:     defb 0
CLRLAST:    defb 0
CLR1:       defb 0
CLR2:       defb 0

FIFO_HEAD:  defb 0
FIFO_TAIL:  defb 0
KEYBUF:     defs 8
SHADOW:     defb 08h

            end
