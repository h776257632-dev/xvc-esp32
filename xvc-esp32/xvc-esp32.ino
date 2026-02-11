/*
 * Description : Robust Xilinx Virtual Cable Server for ESP32
 * Optimized for Stability: Disables WiFi Sleep & TCP Nagle
 */

#include <WiFi.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <netinet/tcp.h> // 必须引入这个以使用 TCP_NODELAY
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <memory>

// ==========================================
// 【请在这里修改您的 WiFi 信息】
// ==========================================
static const char* MY_SSID = "MyHomeWiFi";      // 例如 "MyHomeWiFi"
static const char* MY_PASSPHRASE = "12345678"; // 例如 "12345678"

// ==========================================
// 【引脚定义】
// ==========================================
static constexpr const int tck_gpio = 14; 
static constexpr const int tms_gpio = 15; 
static constexpr const int tdi_gpio = 12; 
static constexpr const int tdo_gpio = 13; 

// ==========================================

#define ERROR_JTAG_INIT_FAILED -1
#define ERROR_OK 1

// Direct GPIO register access for speed
#define GPIO_CLEAR  (GPIO.out_w1tc)
#define GPIO_SET    (GPIO.out_w1ts)
#define GPIO_IN     (GPIO.in)

/* Transition delay coefficients */
// 如果线长或不稳定，适当增加这个值 (例如 20 或 50)
static const unsigned int jtag_delay = 10; 

static bool jtag_read(void);
static void jtag_write(std::uint_fast8_t tck, std::uint_fast8_t tms, std::uint_fast8_t tdi);

static std::uint32_t jtag_xfer(std::uint_fast8_t n, std::uint32_t tms, std::uint32_t tdi)
{
    std::uint32_t tdo = 0;
    const std::uint32_t tdo_bit = (1u << (n - 1));
    for (int i = 0; i < n; i++) {
        jtag_write(0, tms & 1, tdi & 1);
        for (std::uint32_t d = 0; d < jtag_delay; d++) asm volatile ("nop");
        jtag_write(1, tms & 1, tdi & 1);
        for (std::uint32_t d = 0; d < jtag_delay; d++) asm volatile ("nop");
        tdo >>= 1;
        tdo |= jtag_read() ? tdo_bit : 0;
        jtag_write(0, tms & 1, tdi & 1);
        for (std::uint32_t d = 0; d < jtag_delay; d++) asm volatile ("nop");
        tms >>= 1;
        tdi >>= 1;
    }
    return tdo;
}

static bool jtag_read(void)
{
    return !!(GPIO_IN & 1<<tdo_gpio);
}

static void jtag_write(std::uint_fast8_t tck, std::uint_fast8_t tms, std::uint_fast8_t tdi)
{
    const uint32_t set = (tck << tck_gpio) | (tms << tms_gpio) | (tdi << tdi_gpio);
    const uint32_t clear = ((!tck) << tck_gpio) | ((!tms) << tms_gpio) | ((!tdi) << tdi_gpio);

    GPIO_SET = set;
    GPIO_CLEAR = clear;
}

static int jtag_init(void)
{
    GPIO_CLEAR = 1<<tdi_gpio | 1<<tck_gpio;
    GPIO_SET = 1<<tms_gpio;

    pinMode(tdo_gpio, INPUT);
    pinMode(tdi_gpio, OUTPUT);
    pinMode(tck_gpio, OUTPUT);
    pinMode(tms_gpio, OUTPUT);

    jtag_write(0, 1, 0);

    return ERROR_OK;
}

// 优化后的读取函数，处理部分读取情况
static int sread(int fd, void* target, int len) {
   std::uint8_t *t = reinterpret_cast<std::uint8_t*>(target);
   int total = 0;
   while (total < len) {
      int r = read(fd, t + total, len - total);
      if (r <= 0) return r; // 0=closed, -1=error
      total += r;
   }
   return total;
}

static constexpr const char* TAG = "XVC";

struct Socket
{
    int fd;
    Socket() : fd(-1) {}
    Socket(int fd) : fd(fd) {}
    Socket(int domain, int family, int protocol) 
    {
        this->fd = socket(domain, family, protocol);
    }
    Socket(const Socket&) = delete;
    Socket(Socket&& rhs) : fd(rhs.fd) { rhs.fd = -1; }
    ~Socket() { this->release(); }
    void release() { 
        if( this->is_valid() ) {
            closesocket(this->fd);
            this->fd = -1;
        }
    }
    int get() const { return this->fd;}

    Socket& operator=(Socket&& rhs) { 
        if(this->fd != -1) closesocket(this->fd);
        this->fd = rhs.fd; 
        rhs.fd = -1; 
        return *this; 
    }
    bool is_valid() const { return this->fd >= 0; }
};

class XvcServer
{
private:
    Socket listen_socket;
    Socket client_socket;
    unsigned char buffer[4096]; // 稍微加大缓冲区
    unsigned char result[1024]; // 结果缓冲区
public:
    XvcServer(std::uint16_t port) {
        Socket sock(AF_INET, SOCK_STREAM, 0);
        {
            int value = 1;
            setsockopt(sock.get(), SOL_SOCKET, SO_REUSEADDR, &value, sizeof(value));
        }

        sockaddr_in address;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);
        address.sin_family = AF_INET;

        if( bind(sock.get(), reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ) {
            ESP_LOGE(TAG, "Failed to bind socket.");
        }
        if( listen(sock.get(), 1) < 0 ) { // Backlog 1 is enough
            ESP_LOGE(TAG, "Failed to listen the socket.");
        }

        ESP_LOGI(TAG, "Begin XVC Server. port=%d", port);
        this->listen_socket = std::move(sock);
    }

    bool wait_connection()
    {
        if (!this->listen_socket.is_valid()) return false;

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(this->listen_socket.get(), &read_fds);

        timeval timeout; 
        timeout.tv_sec = 0; 
        timeout.tv_usec = 10000; // 10ms check

        if (select(this->listen_socket.get() + 1, &read_fds, 0, 0, &timeout) > 0) {
            if (FD_ISSET(this->listen_socket.get(), &read_fds)) {
                sockaddr_in address;
                socklen_t nsize = sizeof(address);
                int newfd = accept(this->listen_socket.get(), reinterpret_cast<sockaddr*>(&address), &nsize);

                if (newfd >= 0) {
                    // 【关键优化】禁用 Nagle 算法，降低 XVC 协议延迟
                    int flag = 1;
                    setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
                    
                    this->client_socket = Socket(newfd);
                    Serial.println("Client Connected! (Vivado linked)");
                    ESP_LOGI(TAG, "Connection accepted from %s", inet_ntoa(address.sin_addr));
                    return true;
                }
            }
        }
        return false;
    }

    bool handle_data()
    {
        const char xvcInfo[] = "xvcServer_v1.0:2048\n";
        int fd = this->client_socket.get();

        std::uint8_t cmd[16];
        std::memset(cmd, 0, 16);

        // 先读2字节判断命令
        if (sread(fd, cmd, 2) != 2) return false;

        if (memcmp(cmd, "ge", 2) == 0) {
            // getinfo
            if (sread(fd, cmd, 6) != 6) return false;
            if (write(fd, xvcInfo, strlen(xvcInfo)) != (ssize_t)strlen(xvcInfo)) return false;
            return true;
        } else if (memcmp(cmd, "se", 2) == 0) {
            // settck
            if (sread(fd, cmd, 9) != 9) return false;
            memcpy(result, cmd + 5, 4);
            if (write(fd, result, 4) != 4) return false;
            return true;
        } else if (memcmp(cmd, "sh", 2) == 0) {
            // shift
            if (sread(fd, cmd, 4) != 4) return false;
        } else {
            ESP_LOGE(TAG, "invalid cmd");
            return false;
        }

        int len;
        if (sread(fd, &len, 4) != 4) return false;

        int nr_bytes = (len + 7) / 8;
        if (nr_bytes * 2 > (int)sizeof(buffer)) {
            ESP_LOGE(TAG, "Buffer overflow request");
            return false;
        }

        if (sread(fd, buffer, nr_bytes * 2) != nr_bytes * 2) return false;
        memset(result, 0, nr_bytes);
    
        // 执行 JTAG 操作
        jtag_write(0, 1, 1); // Exit-Idle?

        int bytesLeft = nr_bytes;
        int bitsLeft = len;
        int byteIndex = 0;
        uint32_t tdi, tms, tdo;

        while (bytesLeft > 0) {
            tms = 0; tdi = 0; tdo = 0;
            if (bytesLeft >= 4) {
                memcpy(&tms, &buffer[byteIndex], 4);
                memcpy(&tdi, &buffer[byteIndex + nr_bytes], 4);
                tdo = jtag_xfer(32, tms, tdi);
                memcpy(&result[byteIndex], &tdo, 4);
                bytesLeft -= 4; bitsLeft -= 32; byteIndex += 4;
            } else {
                memcpy(&tms, &buffer[byteIndex], bytesLeft);
                memcpy(&tdi, &buffer[byteIndex + nr_bytes], bytesLeft);
                tdo = jtag_xfer(bitsLeft, tms, tdi);
                memcpy(&result[byteIndex], &tdo, bytesLeft);
                bytesLeft = 0;
            }
        }

        jtag_write(0, 1, 0);

        if (write(fd, result, nr_bytes) != nr_bytes) return false;

        return true;
    }

    void run()
    {
        if( this->client_socket.is_valid() ) {
            if( !this->handle_data() ) {
                this->client_socket.release();
                ESP_LOGI(TAG, "Client disconnected or Error.");
                Serial.println("Client Disconnected.");
            }
        }
        else {
            this->wait_connection();
        }
    }
};

static std::unique_ptr<XvcServer> server;

void setup()
{
    jtag_init();

    Serial.begin(115200);
    delay(200);
    Serial.println("\n\nStarting Optimized XVC Server...");

    // 1. 设置 WiFi 模式
    WiFi.mode(WIFI_STA);
    
    // 【关键优化】关闭 WiFi 省电模式！
    // 如果不关这个，Vivado 发包时 ESP32 可能正在睡觉，导致超时
    WiFi.setSleep(false); 

    Serial.print("Connecting to WiFi: ");
    Serial.println(MY_SSID);
    WiFi.begin(MY_SSID, MY_PASSPHRASE);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void loop()
{
    // 简单的连接保持逻辑
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Lost! Reconnecting...");
        WiFi.disconnect();
        WiFi.reconnect();
        while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
        Serial.println("\nReconnected.");
    }

    if( !server ) {
        server.reset(new XvcServer(2542));
    }
    
    // 运行服务
    server->run();
}