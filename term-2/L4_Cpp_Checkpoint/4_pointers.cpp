//
//  4_pointers.cpp
//  cpp-test
//
//  Created by jessica yung on 07/03/2017.
//  Copyright © 2017 Jessica Yung. All rights reserved.
//

#include "4_pointers.hpp"
#include <iostream>
using namespace std;

int pointer() {
    //this is an integer variable with value = 54
    int a = 54;
    
    //this variable points to a. It holds the address of the variable 'a'.
    // In this case, the pointerToA = 0x7fffacf32a68. Your address may be different
    int *pointerToA = &a;
    
    //I can dereference the pointer to get the value stored at the pointer.
    //In this case it is the value of 'a', 54.
    int valueAtPointerToA = *pointerToA;
    
    cout << "Location of a: " << *pointerToA << endl;
    cout << "value of a retrieved from pointer: " << valueAtPointerToA << endl;
    return 0;
}
