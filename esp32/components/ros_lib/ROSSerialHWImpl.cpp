#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#include "esp_system.h"
#include "esp_timer.h"

#include "ros.h"

#define USE_UDP_SOCKET

#define ROSSERIAL_PORT          11411

static const char *TAG = "ROS";

ROSSerialHWImpl::ROSSerialHWImpl()
{
  sock = -1;
  rxfifo_used = 0;
}

// any initialization code necessary to use the serial port
void ROSSerialHWImpl::init(char*& roscorename)
{
  strcpy(roscorehostname,roscorename);
  ESP_LOGI(TAG, "init(%s)",roscorename);

  if( sock != -1 )
  {
    close(sock);
    sock = -1;
  }

  sock = -1;
  tryConnect();
}

bool ROSSerialHWImpl::tryConnect()
{
  if( sock == -1 )
  {
    ESP_LOGV(TAG, "tryConnect() create socket ...");

#ifdef USE_UDP_SOCKET
    rxfifo_used = 0;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if( sock == -1 )
    {
      return false;
    }

    // zero out the structure
    memset((char *) &destAddr, 0, sizeof(destAddr));

    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(ROSSERIAL_PORT);
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    //bind socket to port
    if( bind(sock , (struct sockaddr*)&destAddr, sizeof(destAddr) ) == -1)
    {
      return false;
    }

#else
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
#endif

    if( sock != -1 )
    {
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000;
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));
        timeout.tv_sec = 0;
        timeout.tv_usec = 100;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
    }

    connected = false;
  }

  if(( sock != -1 )&&( connected == false ))
  {
    //destAddr.sin_addr.s_addr = inet_addr(roscorename);
    //inet_aton(roscorehostname,&destAddr.sin_addr.s_addr);

    struct addrinfo* result;
    /* resolve the domain name into a list of addresses */
    int error = getaddrinfo(roscorehostname, NULL, NULL, &result);
    if(error != 0 || result == NULL)
    {
      return false;
    }
    destAddr.sin_addr = ((struct sockaddr_in *)result->ai_addr)->sin_addr;

    //ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(destAddr.sin_addr));
    freeaddrinfo(result);

    char addr_str[128];
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(ROSSERIAL_PORT);
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

    ESP_LOGV(TAG, "tryConnect() %s %s:%d",
      addr_str,
      inet_ntoa(destAddr.sin_addr.s_addr),
      ROSSERIAL_PORT);

#ifdef USE_UDP_SOCKET
    ESP_LOGI(TAG, "tryConnect() connected %s|%s:%d",
      roscorehostname,
      inet_ntoa(destAddr.sin_addr.s_addr),
      ROSSERIAL_PORT);

    connected = true;
#else
    ESP_LOGV(TAG, "tryConnect() try connect");
    int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0)
    {
      close(sock);
      sock = -1;
      connected = false;
    }
    else
    {
      ESP_LOGI(TAG, "tryConnect() connected %s|%s:%d",
        roscorehostname,
        inet_ntoa(destAddr.sin_addr.s_addr),
        ROSSERIAL_PORT);
      connected = true;
    }
#endif
  }
  return connected;
}

// read a byte from the serial port. -1 = failure
int ROSSerialHWImpl::read()
{
  if( tryConnect() == true )
  {
#ifdef USE_UDP_SOCKET
    if( rxfifo_used > 0 )
    {
      rxfifo_used--;
      return rxfifo[rxfifo_rdidx++];
    }
    else
    {
      int len;
      socklen_t slen = sizeof(destAddr);
      len = recvfrom(sock, rxfifo, sizeof(rxfifo), MSG_DONTWAIT, (struct sockaddr *) &destAddr, &slen);
      if( len <= 0 )
      {
        return -1;
  		}
      else
      {
        rxfifo_used = len;
        rxfifo_rdidx = 0;
        //ESP_LOGI(TAG, "read %d bytes read",rxfifo_used);
        rxfifo_used--;
        return rxfifo[rxfifo_rdidx++];
      }
    }
#else
    uint8_t rx_buffer[1];
    int len = recv(sock, rx_buffer, (size_t)sizeof(rx_buffer), MSG_DONTWAIT);
    if( len==-1 && (errno==EAGAIN || errno==EWOULDBLOCK) )
    {
      return -1;
    }
    else if( len < 0 )
    {
      ESP_LOGI(TAG, "read() disconnected err=%d errno=%d",len,errno);
      close(sock);
      sock = -1;
      connected = false;
      return -1;
    }
    else if ( len > 0 )
    {
      //ESP_LOGI(TAG, "read [%d] 0x%02x",len,rx_buffer[0]);
      return rx_buffer[0];
    }
#endif
  }

  return -1;
}

// write data to the connection to ROS
void ROSSerialHWImpl::write(uint8_t* data, int length)
{
  if( tryConnect() == true )
  {
#ifdef USE_UDP_SOCKET
    socklen_t slen = sizeof(destAddr);

#if 0
    ESP_LOGI(TAG, "write %d bytes to %s:%d",
      length,
      inet_ntoa(destAddr.sin_addr.s_addr),
      ROSSERIAL_PORT);
#endif

    if (sendto(sock, data, length, MSG_DONTWAIT, (struct sockaddr*) &destAddr, slen) == -1)
    {
      ESP_LOGI(TAG, "write() sendto error => disconnected");
      close(sock);
      sock = -1;
      connected = false;
    }
#else
    if( send(sock, data, length, 0 /* MSG_DONTWAIT */) != length )
    {
      ESP_LOGI(TAG, "write() send error => disconnected");
      close(sock);
      sock = -1;
      connected = false;
    }
#endif
  }
}

// returns milliseconds since start of program
unsigned long ROSSerialHWImpl::time()
{
  int64_t t = esp_timer_get_time();
  //ESP_LOGI(TAG, "time %d",(int)t/1000);
  return t/1000;
}
