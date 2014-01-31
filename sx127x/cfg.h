/*
 *  This file is part of sx127x_qt.
 *
 *  sx127x_qt is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  sx127x_qt is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with sx127x_qt.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CFG_H
#define CFG_H

#include <stdio.h>
#include <stdint.h>

class Cfg
{
    public:
        Cfg();
        void openFile(char* str);
        void save(char* fname);
        char path[512];
		uint32_t xtal_hz;

    private:
        FILE *f;
        void parseFile(void);
};

#endif // CFG_H
