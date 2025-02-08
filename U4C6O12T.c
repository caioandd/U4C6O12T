#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "pio_matrix.pio.h"

#define I2C_PORT i2c1
#define I2C_SDA 14 //Porta SDA do display
#define I2C_SCL 15 //Porta SCL do display
#define endereco 0x3C //Endereço do display

#define NUM_LEDS 25 //Numero de LEDs
#define MATRIZ_PIN 7 //GPIO
volatile int numero = 11; //Número exibido na matriz WS2812

#define LED_G 11
#define LED_B 12
#define BOTAO_A 5 //GPIO do botão A
#define BOTAO_B 6 //GPIO do botão B

#define DEBOUNCE_DELAY 400 // 200 ms para debouncing
volatile int ultima_interrup = 0; // Para armazenar o último tempo de interrupção

volatile bool estado_g = false;
volatile bool estado_b = false;

ssd1306_t ssd;

//Mapeamento físico dos LEDs
int PHYSICAL_LEDS_MAPPER[25] = {
    24, 23, 22, 21, 20,
    15, 16, 17, 18, 19,
    14, 13, 12, 11, 10,
    5, 6, 7, 8, 9,
    4, 3, 2, 1, 0};
//Vetores dos números
double numero0[25] =
    {0.0, 0.2, 0.2, 0.2, 0.0,  // 0
     0.0, 0.2, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0};
double numero1[25] =
    {0.0, 0.0, 0.2, 0.0, 0.0,  // 1
     0.0, 0.2, 0.2, 0.0, 0.0,
     0.0, 0.0, 0.2, 0.0, 0.0,
     0.0, 0.0, 0.2, 0.0, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0};
double numero2[25] =
    {0.0, 0.2, 0.2, 0.2, 0.0,  // 2
     0.0, 0.0, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0,
     0.0, 0.2, 0.0, 0.0, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0};
double numero3[25] =
    {0.0, 0.2, 0.2, 0.2, 0.0,  // 3
     0.0, 0.0, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0,
     0.0, 0.0, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0};
double numero4[25] =
    {0.0, 0.2, 0.0, 0.2, 0.0,  // 4
     0.0, 0.2, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0,
     0.0, 0.0, 0.0, 0.2, 0.0,
     0.0, 0.0, 0.0, 0.2, 0.0};
double numero5[25] =
    {0.0, 0.2, 0.2, 0.2, 0.0,  // 5
     0.0, 0.2, 0.0, 0.0, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0,
     0.0, 0.0, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0};
double numero6[25] =
    {0.0, 0.2, 0.2, 0.2, 0.0,  // 6
     0.0, 0.2, 0.0, 0.0, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0,
     0.0, 0.2, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0};
double numero7[25] =
    {0.0, 0.2, 0.2, 0.2, 0.0,  // 7
     0.0, 0.0, 0.0, 0.2, 0.0,
     0.0, 0.0, 0.0, 0.2, 0.0,
     0.0, 0.0, 0.2, 0.0, 0.0,
     0.0, 0.0, 0.2, 0.0, 0.0};
double numero8[25] =
    {0.0, 0.2, 0.2, 0.2, 0.0,  // 8
     0.0, 0.2, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0,
     0.0, 0.2, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0};
double numero9[25] =
    {0.0, 0.2, 0.2, 0.2, 0.0,  // 9
     0.0, 0.2, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0,
     0.0, 0.0, 0.0, 0.2, 0.0,
     0.0, 0.2, 0.2, 0.2, 0.0};
double limpa[25] =
    {0.0, 0.0, 0.0, 0.0, 0.0,  // limpa a matriz
     0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0};
//Ponteiro para os vetores dos números
double *nums[]={numero0, numero1, numero2, numero3, numero4, numero5, numero6, numero7, numero8, numero9};

//Chamada primária
void init_gpios();
void padrao(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b);
static void gpio_irq_handler(uint gpio, uint32_t events);

int main()
{
  //Matriz 5x5 WS2812
  PIO pio = pio0; 
  bool ok;
  uint16_t i;
  uint32_t valor_led;
  double r, g, b;

  ok = set_sys_clock_khz(128000, false);

  stdio_init_all();
  init_gpios();

  uint sm = pio_claim_unused_sm(pio, true);
  uint offset = pio_add_program(pio, &pio_matrix_program);
  pio_matrix_program_init(pio, sm, offset, MATRIZ_PIN);

  //Display SSD1306
  i2c_init(I2C_PORT, 400 * 1000); // Clock de 400Khz
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set SDA (dados)
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set SCL (clock)
  gpio_pull_up(I2C_SDA); // Pull up SDA
  gpio_pull_up(I2C_SCL); // Pull up SCL
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display
  ssd1306_fill(&ssd, false);//Limpa o display
  ssd1306_send_data(&ssd);

  //Callback para controle dos LEDs azul e verde
  gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, 1, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, 1, &gpio_irq_handler);

  //Define as cores RGB da matriz WS2812
  r=1;
  g=1;
  b=1;
  padrao(limpa, valor_led, pio, sm, r, g, b); // Limpa matriz WS2812
  ssd1306_draw_string(&ssd, "LED VERDE OFF", 8, 10); // Desenha estado do LED verde (inicial)
  ssd1306_draw_string(&ssd, "LED AZUL OFF", 8, 48); // Desenha estado do LED azul (inicial)
  ssd1306_send_data(&ssd); // Atualiza o display

  while (true)
  {
    char entrada[1];
    scanf("%1s", entrada); //"1s" evita que o buffer fique travado
    ssd1306_draw_string(&ssd, entrada, 64, 30); // Desenha o caractere solicitado     
    if (entrada[0] >= '0' && entrada[0] <= '9')
    {
      int numero = atoi(entrada);
      padrao(nums[numero], valor_led, pio, sm, r, g, b);//Chamada para matriz
    }
    
    ssd1306_send_data(&ssd); // Atualiza o display
    
    sleep_ms(1000);
  }
}
void init_gpios(){ // Inicializa componentes gerais
  gpio_init(BOTAO_A);
  gpio_init(BOTAO_B);
  gpio_init(LED_B);
  gpio_init(LED_G);

  gpio_set_dir(BOTAO_A, GPIO_IN);
  gpio_set_dir(BOTAO_B, GPIO_IN);
  gpio_set_dir(LED_B, GPIO_OUT);
  gpio_set_dir(LED_G, GPIO_OUT);

  gpio_pull_up(BOTAO_A);
  gpio_pull_up(BOTAO_B);
}
//Função para conversão dos valores das cores
uint32_t matrix_rgb(double r, double g, double b){
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G<<24) | (R << 16) | (B << 8);
}
//Função padrão para ligar os LEDs escolhidos
void padrao(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){
    for (int16_t i = 0; i < NUM_LEDS; i++)
    {
        int led_matrix_location = PHYSICAL_LEDS_MAPPER[i];
        valor_led = matrix_rgb(r*desenho[led_matrix_location], g*desenho[led_matrix_location], b*desenho[led_matrix_location]);
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}

static void gpio_irq_handler(uint gpio, uint32_t events){
  uint32_t tempo_interrup = to_ms_since_boot(get_absolute_time()); // Obtém o tempo atual
    if (tempo_interrup - ultima_interrup > DEBOUNCE_DELAY) { // Verifica o tempo de debounce
        ultima_interrup = tempo_interrup; // Atualiza o tempo da última interrupção
      if (gpio_get(BOTAO_A)==0)
      {
        estado_g = !estado_g;
        gpio_put(LED_G, estado_g);
        printf("LED VERDE %s\n", estado_g ? "ON" : "OFF");
          if (estado_g)
          {
            ssd1306_draw_string(&ssd, "LED VERDE ON ", 8, 10); // Desenha estado do LED verde
            ssd1306_send_data(&ssd); // Atualiza o display 
          } else {
            ssd1306_draw_string(&ssd, "LED VERDE OFF", 8, 10); // Desenha estado do LED verde
            ssd1306_send_data(&ssd); // Atualiza o display
          }
      }

      else if (gpio_get(BOTAO_B)==0)
      {
        estado_b = !estado_b;
        gpio_put(LED_B, estado_b);
        printf("LED AZUL %s\n", estado_b ? "ON" : "OFF");
          if (estado_b)
          {
            ssd1306_draw_string(&ssd, "LED AZUL ON ", 8, 48); // Desenha estado do LED azul
            ssd1306_send_data(&ssd); // Atualiza o display 
          } else {
            ssd1306_draw_string(&ssd, "LED AZUL OFF", 8, 48); // Desenha estado do LED azul
            ssd1306_send_data(&ssd); // Atualiza o display
          } 
      }
    }
}