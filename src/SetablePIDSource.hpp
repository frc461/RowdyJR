/*
 * SetablePIDSource.hpp
 *
 *  Created on: Nov 13, 2017
 *      Author: hank
 */
#include "WPILib.h"

class SettablePIDSource : public PIDSource {

private:
	double m_value;

public:

	SettablePIDSource() : m_value(0) { }

	void set(double value) { m_value = value; }
	double PIDGet() {
		return m_value;
	}
};

