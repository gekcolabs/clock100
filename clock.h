/*
clock.h - Header file for the Digital Clock

Version: 1.0
(c) 2018 Thomas Gould
www.gekco.com

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

typedef enum
{
    SET_TIME			= 0x00,
    SET_UTC_ON_OFF		= 0x01,        
    SET_UTC_HOURS		= 0x02,        
    SET_ALARM			= 0x03,
    TOGGLE_ALARM		= 0x04,
    SET_LOCAL_TM12_24		= 0x05,
    SET_DISPLAY_BRIGHTNESS	= 0x06,
} MENU_STATES;