MOVIH $0 0x80 // pointer to switches
LD $1, 0($0) // get switch state
MOVIL $4 0x0F // to clear all but 4 lsb's
AND $2 $1 $4 // first operand in $2
SHIFTR $3 $1
SHIFTR $3 $3
SHIFTR $3 $3
SHIFTR $3 $3
AND $3 $3 $4 // second operand in $3
LOOP: 	ADD $5 $2 $5
	ADDI $6 $6 1 // counter
	BLE $6 $3 LOOP
SUB $5 $5 $2 // correct for overcounting
MOVIH $7 0x90 // pointer to 7Seg
ST $5 0($7)
END: BEQ $2 $2 END // Halt


