# Low-Cost Cubesat Prototype (LCP)
(EN) This repository contains the Hardware and Software Projects to construct and operate a low cost nanosatelite prototype (LCP) developed by the electrical engineering students at the Centro Universitário FEI (Brazil), as part of the Electrical Engineering Bachelor's Thesis Project.

(PT) Este repositório comporta os Projetos de Hardware e Software para construção e operação de um protótipo de baixo custo de nanosatélite (LCP) desenvolvido pelos alunos de engenharia elétrica do Centro Universitério da FEI, como parte do projeto de conclusão de curso de Bacharel em Engenharia Elétrica.

## Hardware Project for the LCP platform
(EN) The hardware project comprehends electrical schematics, PCB designs as well as the BOM (Bill of Materials) intended for its construction.

(PT) O projeto de hardware compreende os esquemas elétricos, design das PCBs e também seu BOM (Lista de Materiais) destinados para sua construção.

<img width="150" height="150" src="https://raw.githubusercontent.com/brn-duarte/LCP-Project/master/Media/20200614_013407.jpg"> <img width="150" height="150" src="https://raw.githubusercontent.com/brn-duarte/LCP-Project/master/Media/20200616_192936.jpg"> <img width="150" height="150" src="https://raw.githubusercontent.com/brn-duarte/LCP-Project/master/Media/20200614_013852.jpg">

## Flight Software for the LCP platform
(EN) The target MCU is the Renesas RL78 - R5F100LE, using the RL78 IAR programming and development tool (IAR IDE for the RL78).
In addition the ground station software is available to RaspberryPi and OrangePi platforms (tests were performed using a Pi Zero W and OrangePi One models).

(PT) O MCU objetivo de utilização foi o Renesas RL78 - R5F100LE, havendo sido utilizado o RL78 IAR para fins de programação e desenvolvimento (IAR IDE for the RL78).
Ainda, o software da estação terrena encontra-se disponível para as plataformas RaspberryPi e OrangePi (testes realizados utilizando os modelos Pi Zero W e OrangePi One).

Execution Example:
```
>>> import lcp

LCP script has started.
Type in:
lcp.diconnect() to disconnect
or CTRL+C to exit.

SX1276 Configuration OK
FLOOR RSSI: - 44.0 dBm

>>> lcp.send_sensor_read ()

INFORMATION FRAME SENT: REQUEST SENSOR READINGS
SEQUENCE NUMBER N(R): 0x1
WAITING FOR REPLY...

MESSAGE RECEIVED (RESPONSE)
REPLY OF INFORMATION FRAME FROM: FEILCP | SEQUENCE N(S): 0x1 | PAYLOAD CODE: 0xa1
SENSORS READING REPLY:
NTC0:             21.81      C
NTC1:             23.30      C
LDR0:             3.13       lux
LDR1:             6.18       lux
LDR2:             2.20       lux
LDR3:             0.09       lux
LDR4:             1.30       lux
LDR5:             4.13       lux
VOLTAGE:          3.28       V
CURRENT:          2.10       mA
BMP280 Temp:      32.04      C
BMP280 Pres:      175.11     kPa
BMP280 Pres:      1.31       mmHg
TIME TO RESPOND:  46.9       ms
RECEIVED SIGNAL STRENGTH INDICATION: - 55.5 dBm
******************************************************************
```

Demonstration: (Watch on YouTube)
[![Low-Cost Cubesat](https://www.esa.int/var/esa/storage/images/esa_multimedia/images/2015/01/picasso_cubesat2/15211126-1-eng-GB/PICASSO_CubeSat_article.jpg)](https://youtu.be/lgCKG8bld8Y "Low-Cost Cubesat Prototype - Click to Watch!")
