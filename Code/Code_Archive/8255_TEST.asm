.ORG 0000H

MAIN:  
	MVI A,092H; CONTROL WORD FOR 8255 - P A&B=IP P C=OP
	OUT 03H; CONTROL REGISTER ADDRESS =03H
	MVI A,0AAH
	OUT 002H


START: RIM 
ANI 080H
ORI 040H
SIM
JMP START