HUERTO INTELIGENTE:

Usado: 	Raspberry Pi Model B x1
	MCU ESP 8266 x3 
	Sensor flujo de agua YF-S201
	Sensor humedad del suelo Moisture Sensor v1.4
	Sensor electrico no intrusivo SCT013 30A/1V

Conexiones:
	Flujo agua (Sensor - MCU)
		5V (Rojo) - Vin 
		Tierra (Negro) - GND
		Dato (Amarillo) - divisor de tension 5V a 3.3V - D1
		Divisor de tensión (2R1=R2 [R1=3k9+5k6[Naranja-Blanco-Rojo Verde-Azul-Rojo] R2=18K [Marron-Gris-Naranja])
	
	Sensor humedad (Sensor - MCU)
		VCC - 3,3
		GND - GND
		SIG - A0
		
Apoyo: 
	https://randomnerdtutorials.com/interrupts-timers-esp8266-arduino-ide-nodemcu/
