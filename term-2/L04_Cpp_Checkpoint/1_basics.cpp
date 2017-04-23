//
//  l1_basics.cpp
//  cpp-test
//
//  Created by jessica yung on 04/03/2017.
//  Copyright © 2017 Jessica Yung. All rights reserved.
//


// #include "l1_basics.hpp"

/*write a C++ program that outputs the following statement:
 *** "Hello world, I am ready for C++"
 */

/* 1.6 */

#include <iostream>

// For formatting. In std.
#include <iomanip>

// For file IO
#include <fstream>

// Tell compiler assume we're using std library
// May be problematic (may ref something else instead) if code is added
// to a large project
using namespace std;

int main() {
    // String literals need double quotes, numbers do not
    std::cout << "Hello world, I am ready for C++";
    cout << 23;
    int int_one = 1235;
    cout << "The value of the integer is " << int_one << "\n";
    cout << "Can also use endl to add newline" << endl;
    
    
    /* Variable types and sizes */
    cout << "int size = " << sizeof(int) << endl;
    cout << "short size = " << sizeof(short) << endl;
    cout << "long size = " << sizeof(long) << endl;
    cout << "char size = " << sizeof(char) << endl;
    cout << "float size = " << sizeof(float) << endl;
    cout << "double size = " << sizeof(double) << endl;
    cout << "bool size = " << sizeof(bool) << endl;
    // 4, 2, 8, 1, 4, 8, 1
    
    
    /* Constant variables: cannot change value during the program.*/
    // Trying to change the const value will return an error.
    const int weightGoal = 100;
    cout<<"WeightGoal = "<<weightGoal<<"\n";

    /* Enumerated constants: Assigning a finite number of values to a variable*/
    //define MONTHS as having 12 possbile values
    // Jan = 0, Feb = 1 etc.
    enum MONTH {Jan, Feb, Mar, Apr,May,Jun,Jul,Aug,Sep,Oct,Nov,Dec};
    
    //define bestMonth as a variable type MONTHS
    MONTH bestMonth;
    
    //assign bestMonth one of the values of MONTHS
    bestMonth = Jan;
    
    //now we can check the value of bestMonths just
    //like any other variable
    if(bestMonth == Jan)
    {
        cout<<"I'm not so sure January is the best month\n";
    }

    
    /* Formatting output */
    // #include <iomanip>
    std::cout<<"\n\nThe text without any formatting\n";
    std::cout<<"Ints"<<"Floats"<<"Doubles"<< "\n";
    
    std::cout<<"\nThe text with setw(15)\n";
    // setw stands for 'set width'
    std::cout<<"Ints"<<std::setw(15)<<"Floats"<<std::setw(15)<<"Doubles"<< "\n";
    
    std::cout<<"\n\nThe text with tabs\n";
    std::cout<<"Ints\t"<<"Floats\t"<<"Doubles"<< "\n";
    
    
    /* Reading and writing files */
    // #include <fstream>
    // ostream: writing file (output stream)
    // fstream: both reading and writing.
    // istream: reading file
    // input and output from the perspective of the program
    
    string line;
    //create an output stream to write to the file
    //append the new lines to the end of the file
    ofstream myfileI ("input.txt", ios::app);
    if (myfileI.is_open())
    {
        myfileI << "\nI am adding a line.\n";
        myfileI << "I am adding another line.\n";
        myfileI.close();
    }
    else cout << "Unable to open file for writing";
    
    //create an input stream to write to the file
    ifstream myfileO ("input.txt");
    if (myfileO.is_open())
    {
        while ( getline (myfileO,line) )
        {
            cout << line << '\n';
        }
        myfileO.close();
    }
    
    else cout << "Unable to open file for reading";


    /* Include your own libraries in a header file */
    // Can put all imports and using in a main.hpp file and put simply
    // #include "main.hpp"
    // and the functions
    // in the main.cpp file

    /* User input */
    int year = 0;
    cout << "What year is it?" << endl;
    // get the user response and assign it to the variable year
    cin >> year;
    cout << "It is " << year << "." << endl;
    // BUT std::cin will not retrieve strings that have a space in them
    string varWithSpaces;
    getline (cin, varWithSpaces);
    cout << "This is your var with spaces: " << varWithSpaces;
    
    return 0;

}

/* To compile:
    g++ main.cpp -o main.out
    compiler_name file_name -o name_of_output

    To run:
    ./main.out
*/



