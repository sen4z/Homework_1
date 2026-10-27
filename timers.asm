.include "m8def.inc"
.equ F_CPU = 8000000
; Начальные интервалы (для 8 МГц это ~2 мс и ~3 мс)
.equ TIMER1_INTERVAL = 16000
.equ TIMER2_INTERVAL = 24000
.equ MASK = 63

.dseg
.org 0x60
; Переменные состояния
period1: .byte 2
period2: .byte 2
length1: .byte 1
length2: .byte 1
rx_head: .byte 1
rx_tail: .byte 1
rx_fault: .byte 1
ev_head: .byte 1
ev_tail: .byte 1
ev_fault: .byte 1
cmd_len: .byte 1
cmd_ready: .byte 1
cmd_bad: .byte 1
rx_discard: .byte 1
tx_busy: .byte 1
tx_pos: .byte 1
pending1: .byte 1
pending2: .byte 1
last_channel: .byte 1

.org 0x100
rx_buffer: .byte 64

.org 0x180
string1: .byte 20
.org 0x194
string2: .byte 20

.org 0x1C0
tx_buffer: .byte 64
.org 0x200
command: .byte 32

.cseg
.org 0
 rjmp reset
.org OC1Aaddr
 rjmp irq_a
.org OC1Baddr
 rjmp irq_b
.org URXCaddr
 rjmp irq_rx
.org UDREaddr
 rjmp irq_tx
.org INT_VECTORS_SIZE

.macro SAVE
 push r16
 in r16,SREG
 push r16
 push r17
 push r18
 push r19
 push r30
 push r31
.endmacro

.macro RESTORE
 pop r31
 pop r30
 pop r19
 pop r18
 pop r17
 pop r16
 out SREG,r16
 pop r16
 reti
.endmacro

reset:
 cli
 ldi r16,high(RAMEND)
 out SPH,r16
 ldi r16,low(RAMEND)
 out SPL,r16
 clr r1
 ldi XL,0x60
 clr XH
 ldi r16,21
clear_state:
 st X+,r1
 dec r16
 brne clear_state

 ; Установка периодов по умолчанию
 ldi r16,low(TIMER1_INTERVAL)
 sts period1,r16
 ldi r16,high(TIMER1_INTERVAL)
 sts period1+1,r16
 ldi r16,low(TIMER2_INTERVAL)
 sts period2,r16
 ldi r16,high(TIMER2_INTERVAL)
 sts period2+1,r16

 ; Длина строк по умолчанию
 ldi r16,6
 sts length1,r16
 sts length2,r16

 ; Копирование строк по умолчанию из Flash в RAM
 ldi ZL,low(default1*2)
 ldi ZH,high(default1*2)
 ldi XL,low(string1)
 ldi XH,high(string1)
 rcall flash_copy

 ldi ZL,low(default2*2)
 ldi ZH,high(default2*2)
 ldi XL,low(string2)
 ldi XH,high(string2)
 rcall flash_copy

 ; Настройка портов (светодиоды для отладки)
 ldi r16,(1<<DDB0)|(1<<DDB1)|(1<<DDB2)
 out DDRB,r16
 out PORTB,r1

 ; Настройка UART
 out UBRRH,r1
 ldi r16,1
 out UBRRL,r16
 ldi r16,(1<<U2X)
 out UCSRA,r16
 ldi r16,(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0)
 out UCSRC,r16
 ldi r16,(1<<RXEN)|(1<<TXEN)|(1<<RXCIE)
 out UCSRB,r16

 ; Настройка Timer1
 out TCCR1B,r1
 out TCNT1H,r1
 out TCNT1L,r1
 ldi r16,high(TIMER1_INTERVAL)
 out OCR1AH,r16
 ldi r16,low(TIMER1_INTERVAL)
 out OCR1AL,r16
 ldi r16,high(TIMER2_INTERVAL)
 out OCR1BH,r16
 ldi r16,low(TIMER2_INTERVAL)
 out OCR1BL,r16
 ldi r16,(1<<COM1A0)|(1<<COM1B0)
 out TCCR1A,r16
 ldi r16,(1<<OCF1A)|(1<<OCF1B)
 out TIFR,r16
 ldi r16,(1<<OCIE1A)|(1<<OCIE1B)
 out TIMSK,r16
 ldi r16,(1<<CS10)
 out TCCR1B,r16

 sei

main:
 rcall receive_one
 lds r16,tx_busy
 tst r16
 brne main

 ; Атомарное извлечение событий таймеров
 in r23,SREG
 cli
 lds r16,pending1
 lds r17,pending2
 mov r18,r16
 or r18,r17
 brne dispatch_pending
 out SREG,r23
 rjmp no_event

dispatch_pending:
 tst r16
 breq dispatch_two
 tst r17
 breq dispatch_one
 lds r18,last_channel
 cpi r18,1
 breq dispatch_two
dispatch_one:
 dec r16
 sts pending1,r16
 ldi r20,1
 rjmp dispatch_done
dispatch_two:
 dec r17
 sts pending2,r17
 ldi r20,2
dispatch_done:
 sts last_channel,r20
 out SREG,r23

 ; Формирование сообщения
 ldi ZL,low(string1)
 ldi ZH,high(string1)
 cpi r20,1
 breq event_string
 ldi ZL,low(string2)
 ldi ZH,high(string2)
event_string:
 rcall tx_begin
copy_event:
 ld r16,Z+
 st X+,r16
 tst r16
 brne copy_event
 rcall tx_start
 rjmp main

no_event:
 lds r16,ev_fault
 tst r16
 breq check_command
 sts ev_fault,r1
 ldi ZL,low(err_queue*2)
 ldi ZH,high(err_queue*2)
 rcall reply_flash
 rjmp main

check_command:
 lds r16,cmd_ready
 tst r16
 breq main
 sts cmd_ready,r1
 rcall execute
 sts cmd_len,r1
 sts cmd_bad,r1
 rjmp main

; ПРЕРЫВАНИЯ

irq_tx:
 SAVE
 rcall transmit_one
 lds r16,tx_busy
 tst r16
 brne irq_tx_done
 cbi UCSRB,UDRIE
irq_tx_done:
 RESTORE

transmit_one:
 lds r16,tx_busy
 tst r16
 breq tx_done
 sbis UCSRA,UDRE
 ret
 lds r17,tx_pos
 ldi ZH,high(tx_buffer)
 ldi ZL,low(tx_buffer)
 add ZL,r17
 adc ZH,r1
 ld r16,Z
 tst r16
 breq tx_finished
 out UDR,r16
 inc r17
 sts tx_pos,r17
 ret
tx_finished:
 sts tx_busy,r1
tx_done:
 ret

tx_begin:
 ldi XL,low(tx_buffer)
 ldi XH,high(tx_buffer)
 ret

tx_start:
 sts tx_pos,r1
 ldi r16,1
 sts tx_busy,r16
 sbi UCSRB,UDRIE
 ret

reply_flash:
 rcall tx_begin
 rcall flash_copy
 rjmp tx_start

flash_copy:
 lpm r16,Z+
 st X+,r16
 tst r16
 brne flash_copy
 ret

irq_rx:
 SAVE
 in r18,UCSRA
 in r16,UDR
 andi r18,(1<<FE)|(1<<DOR)|(1<<PE)
 breq rx_enqueue
 ldi r18,1
 sts rx_fault,r18
 rjmp rx_return
rx_enqueue:
 lds r18,rx_head
 mov r17,r18
 inc r17
 andi r17,MASK
 lds r19,rx_tail
 cp r17,r19
 breq rx_full
 ldi ZH,high(rx_buffer)
 mov ZL,r18
 st Z,r16
 sts rx_head,r17
 rjmp rx_return
rx_full:
 ldi r18,1
 sts rx_fault,r18
rx_return:
 RESTORE

irq_a:
 SAVE
 in r18,OCR1AL
 in r19,OCR1AH
 lds r16,period1
 lds r17,period1+1
 add r18,r16
 adc r19,r17
 out OCR1AH,r19
 out OCR1AL,r18
 ldi r16,1
 rcall enqueue
 RESTORE

irq_b:
 SAVE
 in r18,OCR1BL
 in r19,OCR1BH
 lds r16,period2
 lds r17,period2+1
 add r18,r16
 adc r19,r17
 out OCR1BH,r19
 out OCR1BL,r18
 ldi r16,2
 rcall enqueue
 RESTORE

enqueue:
 ldi ZL,low(pending1)
 ldi ZH,high(pending1)
 cpi r16,1
 breq increment_pending
 adiw ZL,1
increment_pending:
 ld r18,Z
 cpi r18,63
 breq ev_full
 inc r18
 st Z,r18
 ret
ev_full:
 ldi r17,1
 sts ev_fault,r17
 sbi PORTB,PB0
 ret

; ЛОГИКА ОБРАБОТКИ КОМАНД

receive_one:
 lds r16,cmd_ready
 tst r16
 breq rx_process
 ret

rx_process:
 in r19,SREG
 cli
 lds r18,rx_fault
 sts rx_fault,r1
 out SREG,r19
 tst r18
 breq rx_read
 ldi r18,1
 sts rx_discard,r18
 sts cmd_bad,r18
 sts cmd_len,r1
rx_read:
 lds r17,rx_tail
 lds r18,rx_head
 cp r17,r18
 breq receive_done
 ldi ZH,high(rx_buffer)
 mov ZL,r17
 ld r16,Z
 inc r17
 andi r17,MASK
 sts rx_tail,r17
 cpi r16,10
 breq line_end
 lds r18,rx_discard
 tst r18
 brne receive_done
 cpi r16,13
 breq receive_done
 cpi r16,32
 brlo bad_line
 cpi r16,127
 brsh bad_line
 lds r17,cmd_len
 cpi r17,31
 brsh bad_line
 ldi ZL,low(command)
 ldi ZH,high(command)
 add ZL,r17
 adc ZH,r1
 st Z,r16
 inc r17
 sts cmd_len,r17
 ret
bad_line:
 ldi r18,1
 sts cmd_bad,r18
 sts rx_discard,r18
 ret
line_end:
 sts rx_discard,r1
 ldi r18,1
 sts cmd_ready,r18
receive_done:
 ret

execute:
 lds r16,cmd_bad
 tst r16
 breq parse_start
 rjmp fail_input
parse_start:
 lds r21,cmd_len
 tst r21
 brne command_nonempty
 rjmp fail_syntax
command_nonempty:
 lds r20,command
 cpi r21,1
 brne long_command
 cpi r20,'R'
 breq restart
 cpi r20,'?'
 breq status_jump
 rjmp fail_syntax
status_jump:
 rjmp status

long_command:
 cpi r21,3
 brsh length_ok
 rjmp fail_syntax
length_ok:
 lds r22,command+1
 cpi r22,'1'
 breq channel_ok
 cpi r22,'2'
 breq channel_ok
 rjmp fail_syntax
channel_ok:
 lds r16,command+2
 cpi r16,' '
 breq separator_ok
 rjmp fail_syntax
separator_ok:
 cpi r20,'T'
 breq period_jump
 cpi r20,'S'
 breq string_jump
 rjmp fail_syntax
period_jump:
 rjmp set_period
string_jump:
 rjmp set_string

restart:
 in r23,SREG
 cli
 in r18,TCNT1L
 in r19,TCNT1H
 lds r16,period1
 lds r17,period1+1
 add r16,r18
 adc r17,r19
 out OCR1AH,r17
 out OCR1AL,r16
 lds r16,period2
 lds r17,period2+1
 add r16,r18
 adc r17,r19
 out OCR1BH,r17
 out OCR1BL,r16
 ldi r16,(1<<OCF1A)|(1<<OCF1B)
 out TIFR,r16
 sts pending1,r1
 sts pending2,r1
 sts last_channel,r1
 sts ev_fault,r1
 cbi PORTB,PB0
 out SREG,r23
 rjmp success

set_period:
 cpi r21,7
 breq parse_hex
 rjmp fail_syntax
parse_hex:
 ldi ZL,low(command+3)
 ldi ZH,high(command+3)
 clr r24
 clr r25
 ldi r21,4
hex_loop:
 ld r16,Z+
 rcall hex_digit
 brcc hex_ok
 rjmp fail_syntax
hex_ok:
 lsl r24
 rol r25
 lsl r24
 rol r25
 lsl r24
 rol r25
 lsl r24
 rol r25
 or r24,r16
 dec r21
 brne hex_loop

 ; Проверка допустимости
 lds r20,length1
 cpi r22,'1'
 breq validate_period
 lds r20,length2
validate_period:
 rcall admissible
 brcc period_admitted
 rjmp fail_load
period_admitted:
 in r23,SREG
 cli
 cpi r22,'1'
 brne commit_period2
 sts period1,r24
 sts period1+1,r25
 rjmp period_committed
commit_period2:
 sts period2,r24
 sts period2+1,r25
period_committed:
 out SREG,r23
 rjmp success

set_string:
 mov r20,r21
 subi r20,3
 cpi r20,17
 brlo string_length_ok
 rjmp fail_syntax
string_length_ok:
 tst r20
 breq string_prefix_ok
 lds r16,command+3
 cpi r16,'!'
 brne string_prefix_ok
 rjmp fail_syntax
string_prefix_ok:
 lds r24,period1
 lds r25,period1+1
 cpi r22,'1'
 breq validate_string
 lds r24,period2
 lds r25,period2+1
validate_string:
 rcall admissible
 brcc string_admitted
 rjmp fail_load
string_admitted:
 ldi XL,low(string1)
 ldi XH,high(string1)
 cpi r22,'1'
 brne write_string2
 sts length1,r20
 rjmp write_string
write_string2:
 ldi XL,low(string2)
 ldi XH,high(string2)
 sts length2,r20
write_string:
 ldi ZL,low(command+3)
 ldi ZH,high(command+3)
 mov r21,r20
 tst r21
 breq string_end
string_copy:
 ld r16,Z+
 st X+,r16
 dec r21
 brne string_copy
string_end:
 ldi r16,13
 st X+,r16
 ldi r16,10
 st X+,r16
 st X,r1
 rjmp success

admissible:
 mov r18,r20
 subi r18,-2
 lsl r18
 lsl r18
 cpi r18,16
 brsh minimum_ready
 ldi r18,16
minimum_ready:
 cp r25,r18
 brlo inadmissible
 clc
 ret
inadmissible:
 sec
 ret

hex_digit:
 cpi r16,'0'
 brlo hex_invalid
 cpi r16,'9'+1
 brlo hex_decimal
 andi r16,0xDF
 cpi r16,'A'
 brlo hex_invalid
 cpi r16,'F'+1
 brsh hex_invalid
 subi r16,'A'-10
 clc
 ret
hex_decimal:
 subi r16,'0'
 clc
 ret
hex_invalid:
 sec
 ret

status:
 rcall tx_begin
 ldi ZL,low(status1*2)
 ldi ZH,high(status1*2)
 rcall append_flash
 lds r16,period1+1
 rcall hex_byte
 lds r16,period1
 rcall hex_byte
 ldi ZL,low(status2*2)
 ldi ZH,high(status2*2)
 rcall append_flash
 lds r16,period2+1
 rcall hex_byte
 lds r16,period2
 rcall hex_byte
 ldi ZL,low(status3*2)
 ldi ZH,high(status3*2)
 rcall append_flash
 lds r16,length1
 rcall hex_byte
 ldi ZL,low(status4*2)
 ldi ZH,high(status4*2)
 rcall append_flash
 lds r16,length2
 rcall hex_byte
 ldi r16,13
 st X+,r16
 ldi r16,10
 st X+,r16
 st X,r1
 rjmp tx_start

append_flash:
 lpm r16,Z+
 tst r16
 breq appended
 st X+,r16
 rjmp append_flash
appended:
 ret

hex_byte:
 mov r17,r16
 swap r16
 andi r16,15
 rcall hex_out
 mov r16,r17
 andi r16,15
hex_out:
 cpi r16,10
 brlo hex_out_decimal
 subi r16,-('A'-10)
 st X+,r16
 ret
hex_out_decimal:
 subi r16,-'0'
 st X+,r16
 ret

success:
 ldi ZL,low(ok*2)
 ldi ZH,high(ok*2)
 rjmp reply_flash
fail_syntax:
 ldi ZL,low(err_syntax*2)
 ldi ZH,high(err_syntax*2)
 rjmp reply_flash
fail_input:
 ldi ZL,low(err_input*2)
 ldi ZH,high(err_input*2)
 rjmp reply_flash
fail_load:
 ldi ZL,low(err_load*2)
 ldi ZH,high(err_load*2)
 rjmp reply_flash

; Строки по умолчанию
default1: .db "ping\r\n",0,0
default2: .db "pong\r\n",0,0
ok: .db "!OK\r\n",0
err_syntax: .db "!ERR SYNTAX\r\n",0
err_input: .db "!ERR INPUT\r\n",0,0
err_load: .db "!ERR LOAD\r\n",0
err_queue: .db "!ERR QUEUE\r\n",0,0
status1: .db "!OK T1=",0
status2: .db " T2=",0,0
status3: .db " L1=",0,0
status4: .db " L2=",0,0