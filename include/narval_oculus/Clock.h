/******************************************************************************
 * narval_oculus driver library for Blueprint Subsea Oculus sonar.
 * Copyright (C) 2020 ENSTA-Bretagne
 * 
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef _DEF_NARVAL_OCULUS_CLOCK_H_
#define _DEF_NARVAL_OCULUS_CLOCK_H_

#include <iostream>
#include <chrono>

namespace narval { namespace oculus {

/** 
 * Simple type to measure time less verbose than std::chrono.
 */
class Clock
{
    protected:

    std::chrono::time_point<std::chrono::high_resolution_clock> t0_;

    public:

    Clock() { this->reset(); }
    
    void reset() { t0_ = std::chrono::high_resolution_clock::now(); }
    
    /**
     * Return current time relative to epoch t0_.
     * (Successive calls to interval will give ellapsed time since last reset).
     */
    template<typename T = double>
    T now() const
    {
        return std::chrono::duration<T>(
            std::chrono::high_resolution_clock::now() - t0_).count();
    } 

    /**
     * Return current time relative to epoch t0_, and set epoch t0_ to now.
     * (Successive calls to interval will give ellapsed time since last call).
     */
    template<typename T = double>
    T interval()
    {
        T res;
        auto t = std::chrono::high_resolution_clock::now();
        res = std::chrono::duration<T>(t - t0_).count();
        t0_ = t;

        return res;
    } 
};

}; //namespace oculus
}; //namespace narval

inline std::ostream& operator<<(std::ostream& os, const narval::oculus::Clock& clock)
{
    os << clock.now() << "s";
    return os;
}

#endif //_DEF_NARVAL_OCULUS_CLOCK_H_
