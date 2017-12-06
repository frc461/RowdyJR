/*
 * SetablePIDOut.cpp
 *
 *  Created on: Nov 13, 2017
 *      Author: hank
 */

#include <SetablePIDOut.h>

SetablePIDOut::SetablePIDOut() {
	output = 0;
}

SetablePIDOut::~SetablePIDOut() {
	// TODO Auto-generated destructor stub
}

void SetablePIDOut::PIDWrite(double output) {
	this->output=output;
}

