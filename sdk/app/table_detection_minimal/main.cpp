/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header


bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
    signal(SIGINT, ctrlc);
    const char * tty = argc>1 ? argv[1] : "/dev/ttyUSB0";
    _u32         baudrate = 115200;

	rp::standalone::rplidar::RPlidarDriver * driver = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    driver->connect(tty, baudrate);
    driver->startMotor();
    driver->startScan(0,1);

    while (1) {
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = sizeof(nodes);

        if (driver->grabScanDataHq(nodes, count) == 0) {
            driver->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                // Get the height of angle closest to zero
                int zeroHeight = nodes[0].angle_z_q14 * 90.f < (360 - nodes[count-1].angle_z_q14 * 90.f) ? (nodes[0].dist_mm_q2/4.0f) : (nodes[count-1].dist_mm_q2/4.0f);
                printf("zeroHeight: %d \r", zeroHeight);
            }
        }

        if (ctrl_c_pressed) break; 
    }

    driver->stop();
    driver->stopMotor();
}

