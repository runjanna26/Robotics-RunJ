#ifndef HELLO_RUN_J_H
#define HELLO_RUN_J_H

class HelloRunJ {
public:
	HelloRunJ();
	virtual ~HelloRunJ();
	void setSomeVariable(int value); // Setter method
	int getSomeVariable() const; // Getter method for debugging

private:
	int some_variable;
};

#endif // HELLO_RUN_J_H
