#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <vector>
#include <string>
#include <deque>
#include <numeric>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <glob.h>

class GpsNode : public rclcpp::Node
{
public:
    GpsNode() : Node("gps_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GpsNode::timer_callback, this));
            
        // Buffer de 100 muestras como pidió el usuario para mayor suavizado
        buffer_size_ = 100;
        
        RCLCPP_INFO(this->get_logger(), "🚀 Nodo GPS (C++) iniciado. Buffer: %zu muestras", buffer_size_);
    }

    ~GpsNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    void timer_callback()
    {
        if (serial_fd_ < 0) {
            connect_serial();
            return;
        }

        read_serial_data();
    }

    void connect_serial()
    {
        // Buscar puertos
        std::vector<std::string> ports;
        glob_t glob_result;
        glob("/dev/ttyUSB*", GLOB_TILDE, NULL, &glob_result);
        for(unsigned int i=0; i<glob_result.gl_pathc; ++i){
            ports.push_back(std::string(glob_result.gl_pathv[i]));
        }
        globfree(&glob_result);
        
        glob("/dev/ttyACM*", GLOB_TILDE, NULL, &glob_result);
        for(unsigned int i=0; i<glob_result.gl_pathc; ++i){
            ports.push_back(std::string(glob_result.gl_pathv[i]));
        }
        globfree(&glob_result);

        if (ports.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "❌ No se encontraron puertos seriales (/dev/ttyUSB* o ACM*)");
            return;
        }

        for (const auto& port : ports) {
            int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (fd < 0) {
                RCLCPP_WARN(this->get_logger(), "❌ No se pudo abrir %s (Error %d: %s). Verifica permisos o si está ocupado.", 
                    port.c_str(), errno, strerror(errno));
                continue;
            }
            
            if (fd >= 0) {
                // Configurar serial 9600 8N1
                struct termios tty;
                tcgetattr(fd, &tty);
                cfsetospeed(&tty, B9600);
                cfsetispeed(&tty, B9600);
                tty.c_cflag &= ~PARENB;
                tty.c_cflag &= ~CSTOPB;
                tty.c_cflag &= ~CSIZE;
                tty.c_cflag |= CS8;
                tty.c_cflag |= CREAD | CLOCAL;
                tty.c_lflag &= ~ICANON; // Raw mode
                tty.c_lflag &= ~ECHO; 
                tty.c_lflag &= ~ECHOE; 
                tty.c_lflag &= ~ISIG;
                tty.c_iflag &= ~(IXON | IXOFF | IXANY);
                tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 
                tcsetattr(fd, TCSANOW, &tty);
                
                // Probar si envia NMEA (Intentar varias lecturas)
                int attempts = 10;
                bool found = false;
                while(attempts-- > 0) {
                    char buf[256];
                    int n = read(fd, buf, sizeof(buf)-1);
                    if (n > 0) {
                        buf[n] = 0;
                        std::string data(buf);
                        if (data.find("$G") != std::string::npos) {
                            serial_fd_ = fd;
                            port_name_ = port;
                            RCLCPP_INFO(this->get_logger(), "✅ GPS conectado en: %s", port.c_str());
                            found = true;
                            break;
                        }
                    }
                    usleep(100000); // Esperar 100ms
                }
                
                if (found) return;
                close(fd);
            }
        }
    }

    void read_serial_data()
    {
        char buf[512];
        int n = read(serial_fd_, buf, sizeof(buf)-1);
        if (n > 0) {
            buf[n] = 0;
            serial_buffer_ += buf;
            
            size_t pos;
            while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
                std::string line = serial_buffer_.substr(0, pos);
                serial_buffer_.erase(0, pos + 1);
                
                // Limpiar CR
                if (!line.empty() && line.back() == '\r') line.pop_back();
                
                // DEBUG TEMPORAL
                // RCLCPP_INFO(this->get_logger(), "RAW: %s", line.c_str());
                
                parse_nmea(line);
            }
        } else if (n < 0 && errno != EAGAIN) {
             RCLCPP_ERROR(this->get_logger(), "Error leyendo serial. Desconectando.");
             close(serial_fd_);
             serial_fd_ = -1;
        }
    }

    void parse_nmea(const std::string& line)
    {
        if (line.find("$GNGGA") == 0 || line.find("$GPGGA") == 0) {
            // Ejemplo: $GNGGA,time,lat,NS,lon,EW,fix,sats,hdop,alt,M...
            std::stringstream ss(line);
            std::string segment;
            std::vector<std::string> parts;
            
            while(std::getline(ss, segment, ',')) {
                parts.push_back(segment);
            }
            
            if (parts.size() < 10) return;
            
            // parts[2] = Lat (ddmm.mmmmm), parts[3] = N/S
            // parts[4] = Lon (dddmm.mmmmm), parts[5] = E/W
            // parts[6] = Fix (1=GPS, 2=DGPS)
            // parts[9] = Alt
            
            try {
                int fix = 0;
                if (!parts[6].empty()) fix = std::stoi(parts[6]);
                
                if (fix > 0 && !parts[2].empty() && !parts[4].empty()) {
                    // Parse lat
                    double lat_raw = std::stod(parts[2]);
                    int lat_deg = (int)(lat_raw / 100);
                    double lat_min = lat_raw - (lat_deg * 100);
                    double lat = lat_deg + (lat_min / 60.0);
                    if (parts[3] == "S") lat = -lat;
                    
                    // Parse lon
                    double lon_raw = std::stod(parts[4]);
                    int lon_deg = (int)(lon_raw / 100);
                    double lon_min = lon_raw - (lon_deg * 100);
                    double lon = lon_deg + (lon_min / 60.0);
                    if (parts[5] == "W") lon = -lon;
                    
                    // Parse alt
                    double alt = 0.0;
                    if (!parts[9].empty()) alt = std::stod(parts[9]);
                    
                    // Agregar al buffer
                    lat_buffer_.push_back(lat);
                    lon_buffer_.push_back(lon);
                    alt_buffer_.push_back(alt);
                    
                    if (lat_buffer_.size() > buffer_size_) lat_buffer_.pop_front();
                    if (lon_buffer_.size() > buffer_size_) lon_buffer_.pop_front();
                    if (alt_buffer_.size() > buffer_size_) alt_buffer_.pop_front();
                    
                    // Calcular promedio
                    double avg_lat = std::accumulate(lat_buffer_.begin(), lat_buffer_.end(), 0.0) / lat_buffer_.size();
                    double avg_lon = std::accumulate(lon_buffer_.begin(), lon_buffer_.end(), 0.0) / lon_buffer_.size();
                    double avg_alt = std::accumulate(alt_buffer_.begin(), alt_buffer_.end(), 0.0) / alt_buffer_.size();
                    
                    auto msg = sensor_msgs::msg::NavSatFix();
                    msg.header.stamp = this->now();
                    msg.header.frame_id = "gps";
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    msg.latitude = avg_lat;
                    msg.longitude = avg_lon;
                    msg.altitude = avg_alt;
                    
                    publisher_->publish(msg);
                    
                    RCLCPP_INFO(this->get_logger(), "🎯 FILTERED (C++): Lat=%.6f, Lon=%.6f, Alt=%.1fm (Sats: %zu)", 
                        avg_lat, avg_lon, avg_alt, lat_buffer_.size());
                } else {
                     RCLCPP_WARN(this->get_logger(), "⚠️ GPS sin fix válido (C++)");
                }
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing NMEA: %s", line.c_str());
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_fd_ = -1;
    std::string port_name_;
    std::string serial_buffer_;
    
    size_t buffer_size_ = 100;
    std::deque<double> lat_buffer_;
    std::deque<double> lon_buffer_;
    std::deque<double> alt_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNode>());
    rclcpp::shutdown();
    return 0;
}
