#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

// C headers for serial port and popen
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>

/**
 * @class SerialPort
 * @brief Manages opening, configuring, and reading from a serial port.
 */
class SerialPort {
private:
    int fd_ = -1;

public:
    SerialPort(const std::string& port_name, speed_t baud_rate) {
        fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ == -1) {
            throw std::runtime_error("Error opening serial port: " + port_name);
        }

        struct termios options;
        tcgetattr(fd_, &options);
        cfsetispeed(&options, baud_rate);
        cfsetospeed(&options, baud_rate);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN]  = 1;
        options.c_cc[VTIME] = 5;
        tcsetattr(fd_, TCSANOW, &options);
    }

    ~SerialPort() {
        if (fd_ != -1) {
            close(fd_);
        }
    }

    std::string read_line() {
        std::string line;
        char ch;
        while (read(fd_, &ch, 1) > 0) {
            if (ch == '\n') break;
            if (ch != '\r') line += ch;
        }
        return line;
    }
};

/**
 * @class Gnuplot
 * @brief Manages a pipe to a gnuplot process for true flicker-free plotting.
 */
class Gnuplot {
private:
    FILE* pipe_ = nullptr;

public:
    Gnuplot() {
        pipe_ = popen("gnuplot -persist", "w");
        if (!pipe_) {
            throw std::runtime_error("Error opening gnuplot pipe.");
        }
        // ONE-TIME SETUP: Configure the terminal for smooth updates. This is CRITICAL.
        fprintf(pipe_, "set term qt noraise\n");
    }

    ~Gnuplot() {
        if (pipe_) {
            pclose(pipe_);
        }
    }

    // This single method handles a complete redraw for each frame.
    void plot(const std::vector<std::vector<double>>& data, int data_index, int max_points) {
        // Step 1: Define all datablocks with the latest data.
        fprintf(pipe_, "$AccelX << EOD\n");
        for (int i = 0; i < max_points; ++i) fprintf(pipe_, "%d %f\n", i, data[0][(data_index + i) % max_points]);
        fprintf(pipe_, "EOD\n");

        fprintf(pipe_, "$AccelY << EOD\n");
        for (int i = 0; i < max_points; ++i) fprintf(pipe_, "%d %f\n", i, data[1][(data_index + i) % max_points]);
        fprintf(pipe_, "EOD\n");

        fprintf(pipe_, "$AccelZ << EOD\n");
        for (int i = 0; i < max_points; ++i) fprintf(pipe_, "%d %f\n", i, data[2][(data_index + i) % max_points]);
        fprintf(pipe_, "EOD\n");

        fprintf(pipe_, "$GyroX << EOD\n");
        for (int i = 0; i < max_points; ++i) fprintf(pipe_, "%d %f\n", i, data[3][(data_index + i) % max_points]);
        fprintf(pipe_, "EOD\n");

        fprintf(pipe_, "$GyroY << EOD\n");
        for (int i = 0; i < max_points; ++i) fprintf(pipe_, "%d %f\n", i, data[4][(data_index + i) % max_points]);
        fprintf(pipe_, "EOD\n");

        fprintf(pipe_, "$GyroZ << EOD\n");
        for (int i = 0; i < max_points; ++i) fprintf(pipe_, "%d %f\n", i, data[5][(data_index + i) % max_points]);
        fprintf(pipe_, "EOD\n");

        // Step 2: Issue a complete, self-contained multiplot command block.
        // This clears the previous frame and draws the new one cleanly.
        fprintf(pipe_, "set multiplot layout 2,1 title 'Real-Time MPU6050 Sensor Data'\n");
        
        fprintf(pipe_, "set ylabel 'Acceleration (g)'\n");
        fprintf(pipe_, "plot '$AccelX' with lines title 'Accel X', '$AccelY' with lines title 'Accel Y', '$AccelZ' with lines title 'Accel Z'\n");
        
        fprintf(pipe_, "set xlabel 'Sample Number'\n");
        fprintf(pipe_, "set ylabel 'Angular Velocity (deg/s)'\n");
        fprintf(pipe_, "plot '$GyroX' with lines title 'Gyro X', '$GyroY' with lines title 'Gyro Y', '$GyroZ' with lines title 'Gyro Z'\n");
        
        fprintf(pipe_, "unset multiplot\n");
        
        // Step 3: Flush the commands to gnuplot to execute the drawing.
        fflush(pipe_);
    }
};

int main() {
    const std::string SERIAL_PORT = "/dev/ttyACM0";
    const int MAX_DATA_POINTS = 100;

    try {
        SerialPort serial(SERIAL_PORT, B115200);
        Gnuplot gnuplot_process;

        std::vector<std::vector<double>> data(6, std::vector<double>(MAX_DATA_POINTS, 0.0));
        int data_index = 0;

        std::cout << "Starting to read from " << SERIAL_PORT << "... Waiting for data." << std::endl;

        while (true) {
            std::string line = serial.read_line();
            if (line.empty()) {
                continue; // Patiently wait for data from ESP32 after reset
            }

            int items_scanned = sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf",
                   &data[0][data_index], &data[1][data_index], &data[2][data_index],
                   &data[3][data_index], &data[4][data_index], &data[5][data_index]
            );

            if (items_scanned == 6) {
                data_index = (data_index + 1) % MAX_DATA_POINTS;
                gnuplot_process.plot(data, data_index, MAX_DATA_POINTS);
            }
        }
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}