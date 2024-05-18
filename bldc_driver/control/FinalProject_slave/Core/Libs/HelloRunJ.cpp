#include "HelloRunJ.h"

HelloRunJ::HelloRunJ() :
		some_variable(0) {
}

HelloRunJ::~HelloRunJ() {
}

void HelloRunJ::setSomeVariable(int value) {
	some_variable = value;
}

int HelloRunJ::getSomeVariable() const {
	return some_variable;
}
