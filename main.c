#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#define BAUD 9600
#include <util/setbaud.h>

// #define F_CPU 8000000UL

// Initialize the UART
void initUART(void) {
    // Set the baud rate (uses values from setbaud.h with F_CPU defined)
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif

    // Enable transmitter
    UCSR0B = (1 << TXEN0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Function to send a byte of data over UART
void UART_sendByte(uint8_t data) {
    // Wait until transmission buffer is empty
    while (!(UCSR0A & (1 << UDRE0)));
    // Send data
    UDR0 = data;
}

// Function to send a string over UART
void UART_sendString(const char* str) {
    while (*str) {
        UART_sendByte(*str++);
    }
}

// Initialize PWM on PD5 (OC0B), PD6 (OC0A), and PB1 (OC1A)
void initPWM() {
    // For PD5 (OC0B) and PD6 (OC0A)
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01); // Fast PWM mode
    TCCR0B = (1 << CS00); // No prescaling

    // For PB1 (OC1A)
    TCCR1A = (1 << COM1A1) | (1 << WGM10);  // Fast PWM, 8-bit
    TCCR1B = (1 << WGM12) | (1 << CS10);   // No prescaling

    // Set PD5, PD6, and PB1 as output
    DDRD |= (1 << PD5) | (1 << PD6);
    DDRB |= (1 << PB1);
}

// Set PWM duty cycle for OC0B (D5)
void setPWMDutyD5(uint8_t duty) {
    OCR0B = duty;
}

// Set PWM duty cycle for OC0A (D6)
void setPWMDutyD6(uint8_t duty) {
    OCR0A = duty;
}

// Set PWM duty cycle for OC1A (D9)
void setPWMDutyD9(uint8_t duty) {
    OCR1A = duty;
}

// Initialize ADC
void initADC() {
    // Set ADC prescaler to 64 (for 8MHz clock)
    // This gives 125kHz ADC clock speed
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1);
    
    // Set reference voltage to AVCC and left adjust ADC result
    ADMUX |= (1 << REFS0);
    
    // Enable ADC
    ADCSRA |= (1 << ADEN);
}

uint16_t readADC(uint8_t channel) {
    // Ensure channel number is within valid range (0 to 7 for ATmega328p)
    channel &= 0x07;
    
    // Clear the existing channel selection and set the desired channel
    ADMUX = (ADMUX & 0xF8) | channel;
    
    // Start ADC conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    return ADC;
}

// ...

int main(void) {
    initPWM();
    initADC();
    initUART();
    
    uint8_t desiredADCChannel = 0;  // e.g., 0 for A0, 1 for A1, and so on
    
    while (1) {
        // Read value from the desired ADC channel (0 to 1023)
        uint16_t adcValue = readADC(desiredADCChannel);
        
        // Convert ADC value to PWM duty cycle (0 to 255)
        uint8_t pwmDuty = adcValue / 4;
        
        setPWMDutyD5(pwmDuty);
        setPWMDutyD6(pwmDuty);
        setPWMDutyD9(pwmDuty);

        // Send ADC value over UART
        char buffer[5];  // 1023 is a 4-digit number, +1 for null-terminator
        sprintf(buffer, "%u", adcValue);
        UART_sendString(buffer);
        UART_sendString("\r\n");  // Newline for clarity
        
        _delay_ms(10);  // Small delay before next reading
    }
    
    return 0;
}