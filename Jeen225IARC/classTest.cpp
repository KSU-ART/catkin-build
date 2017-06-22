#include <iostream>
#include "color.h"

using namespace std;

int main(){
	Color blob('g');
	cout <<blob.lMin <<" , " <<blob.aMin <<" , " <<blob.bMin <<" , " <<blob.lMax <<" , " <<blob.aMax <<" , " <<blob.bMax <<endl;
}
