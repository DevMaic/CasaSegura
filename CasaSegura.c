#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#define BUZZER_PIN 21 // Defina o pino do buzzer
#define BUZZER_FREQUENCY 2000 // Frequência desejada para o buzzer (em Hz)
#define LED_PIN 12 // Define o pino do LED
#define WIFI_SSID "FIBRANET.COM - 87981616006"  // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASS "Bira1210" // Substitua pela senha da sua rede Wi-Fi

bool alarmando = false;

// Buffer para respostas HTTP
#define HTTP_RESPONSE "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" \
                      "<!DOCTYPE html><html><body>" \
                      "<h1>Controle do LED</h1>" \
                      "<p><a href=\"/led/on\">Ligar LED</a></p>" \
                      "<p><a href=\"/led/off\">Desligar LED</a></p>" \
                      "</body></html>\r\n"

void pwm_init_buzzer(uint pin) {
    // Configura o pino para a função PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Obtém o slice de PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configura o PWM com a frequência desejada
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Ajusta o divisor de clock conforme necessário
    pwm_init(slice_num, &config, true);

    // Define o nível inicial do PWM (0 desliga o buzzer)
    pwm_set_gpio_level(pin, 0);
}

void set_buzzer_frequency(uint pin, uint32_t freq) {
    // Obtém o slice de PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Calcula o contador de wrap para a frequência desejada
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t divider16 = (clock << 4) / freq;
    uint32_t wrap = (divider16 + (1 << 4) - 1) >> 4;

    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), wrap / 8); // 12,5% duty cycle
    pwm_set_enabled(slice_num, true);
}

// Função de callback para processar requisições HTTP
static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        // Cliente fechou a conexão
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Processa a requisição HTTP
    char *request = (char *)p->payload;

    if (strstr(request, "GET /led/on")) {
        gpio_put(LED_PIN, 0);  // Desliga o LED
        alarmando = false;
        set_buzzer_frequency(BUZZER_PIN, 0);
    } else if (strstr(request, "GET /led/off")) {
        gpio_put(LED_PIN, 0);  // Desliga o LED
    }

    // Envia a resposta HTTP
    tcp_write(tpcb, HTTP_RESPONSE, strlen(HTTP_RESPONSE), TCP_WRITE_FLAG_COPY);

    // Libera o buffer recebido
    pbuf_free(p);

    return ERR_OK;
}

// Callback de conexão: associa o http_callback à conexão
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_callback);  // Associa o callback HTTP
    return ERR_OK;
}

// Função de setup do servidor TCP
static void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Erro ao criar PCB\n");
        return;
    }

    // Liga o servidor na porta 80
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }

    pcb = tcp_listen(pcb);  // Coloca o PCB em modo de escuta
    tcp_accept(pcb, connection_callback);  // Associa o callback de conexão

    printf("Servidor HTTP rodando na porta 80...\n");
}

void gerenciarEstadoAlarme(uint gpio, uint32_t events) {
    if(gpio == 5) {
        alarmando = true;
        gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
        pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_PIN), true);
        set_buzzer_frequency(BUZZER_PIN, BUZZER_FREQUENCY);
    } else if(gpio == 6) {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_PIN), false);
        gpio_set_function(BUZZER_PIN, GPIO_FUNC_SIO);
        gpio_set_dir(BUZZER_PIN, GPIO_OUT);
        gpio_put(BUZZER_PIN, 0);

        // Garante que o pino está em nível baixo
        alarmando = false;
    }
    
}

int main() {
    stdio_init_all();  // Inicializa a saída padrão
    sleep_ms(10000);
    printf("Iniciando servidor HTTP\n");

    // Inicializa o Wi-Fi
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Falha ao conectar ao Wi-Fi\n");
        return 1;
    }else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("Endereço IP %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    printf("Wi-Fi conectado!\n");
    printf("Para ligar ou desligar o LED acesse o Endereço IP seguido de /led/on ou /led/off\n");

    // Configura o LED como saída
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Inicia o servidor HTTP
    start_http_server();

    // Inicializa o PWM no pino do buzzer
    pwm_init_buzzer(BUZZER_PIN);

    // Define a frequência do buzzer
    // set_buzzer_frequency(BUZZER_PIN, BUZZER_FREQUENCY);

    gpio_init(5);
    gpio_set_dir(5, GPIO_IN);
    gpio_pull_up(5);

    gpio_init(6);
    gpio_set_dir(6, GPIO_IN);
    gpio_pull_up(6);

    gpio_set_irq_enabled_with_callback(5, true, GPIO_IRQ_EDGE_FALL, &gerenciarEstadoAlarme);
    gpio_set_irq_enabled_with_callback(6, true, GPIO_IRQ_EDGE_FALL, &gerenciarEstadoAlarme);

    // Loop principal
    while (true) {
        // cyw43_arch_poll();  // Necessário para manter o Wi-Fi ativo
        gpio_put(LED_PIN, 1);
        if(alarmando) {
              // Liga o LED
            for (int freq = 0; freq <= 2000; freq += 50) {  // Sobe de tom
                set_buzzer_frequency(BUZZER_PIN, BUZZER_FREQUENCY+freq);
                sleep_ms(40);
            }
            for (int freq = 2000; freq >= 0; freq -= 50) {  // Desce de tom
                set_buzzer_frequency(BUZZER_PIN, BUZZER_FREQUENCY+freq);
                sleep_ms(40);
            }
        }
        sleep_ms(100);
    }

    cyw43_arch_deinit();  // Desliga o Wi-Fi (não será chamado, pois o loop é infinito)
    return 0;
}
