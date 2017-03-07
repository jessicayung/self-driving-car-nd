//
//  3_control_flow.cpp
//  cpp-test
//
//  Created by jessica yung on 06/03/2017.
//  Copyright © 2017 Jessica Yung. All rights reserved.
//

#include "3_control_flow.hpp"

#include<iostream>
#include<string>

int relationalOperator() {
    //instead of printing 0 and 1, create an array where
    //0 = False, 1 = True
    std::string TorF[] = {"False", "True"};
    
    int a = 100;
    int b = 33;
    int c = 33;
    
    //Print out the string values of each relational operation
    std::cout<<"a < b is "<<TorF[a<b];
    std::cout<<"\na > b is "<<TorF[a>b];
    std::cout<<"\na != b is "<<TorF[a!=b];
    std::cout<<"\nc >= b is "<<TorF[c>=b];
    std::cout<<"\nc <= b is "<<TorF[c<=b];
    return 0;
}

int logicalOperators() {
    int A = 5;
    int B = 4;
    int C = 5;
    int D = 0;
    
    std::string TorF[] = {"False", "True"};
    
    //The && operator
    std::cout<<"A == C is "<<TorF[A==C];
    std::cout<<"\n(B == D) is "<<TorF[B==D];
    std::cout<<"\n(B > D) is "<<TorF[B>D];
    //A true && false = false
    std::cout<<"\n\n(A ==C) && (B == D) is "<<TorF[(A ==C) && (B == D)];
    //A true and true = true
    std::cout<<"\n(A ==C) && (B > D) is "<<TorF[(A ==C) && (B > D)];
    
    //The || operator
    //A true || false = true
    std::cout<<"\n\n(A ==C) || (B == D) is "<<TorF[(A ==C) || (B == D)];
    //A true || true = true
    std::cout<<"\n(A ==C) || (B > D) is "<<TorF[(A ==C) || (B > D)];
    
    //The 'Not' operator
    std::cout<<"\n\nA < B is "<<TorF[A<B];
    std::cout<<"\n!(A < B) is "<<TorF[!(A<B)];
    
    std::cout<<"\n\nA == C is "<<TorF[A==C];
    std::cout<<"\n!(A == C) is "<<TorF[!(A==C)];
    
    return 0;
}

int ifStatement() {
    int TARGET = 33;
    int guess;
    std::cout<<"Guess a number between 0 - 100\n";
    std::cin>>guess;
    
    std::cout<<"You guessed: "<<guess<<"\n";
    
    
    if(guess < TARGET)
    {
        std::cout<<"Your guess is too low.\n";
    }
    else if(guess > TARGET)
    {
        std::cout<<"Your guess is too high.\n";
    }
    else
    {
        std::cout<<"Yay! You guessed correctly.\n";
    }
    
    return 0;
}

int switchStatement()
// Note break statements are optional.
{
    int menuItem = 1;
    
    std::cout<<"What is your favorite winter sport?: \n";
    std::cout<<"1.Skiing\n2: Sledding\n3: Sitting by the fire";
    std::cout<<"\n4.Drinking hot chocolate\n";
    std::cout<<"\n\n";
    
    switch(menuItem)
    {
        case(1): std::cout<<"Skiing?! Sounds dangerous!\n";
            break;
        case(2): std::cout<<"Sledding?! Sounds like work!\n";
            break;
        case(3): std::cout<<"Sitting by the fire?! Sounds warm!\n";
            break;
        case(4): std::cout<<"Hot chocolate?! Yum!\n";
            break;
    }
    
    char begin;
    std::cout<<"\n\nWhere do you want to begin?\n";
    std::cout<<"B. At the beginning?\nM. At the middle?";
    std::cout<<"\nE. At the end?\n\n";
    begin = 'M';
    
    switch(begin)
    {
        case('B'): std::cout<<"Once upon a time there was a wolf.\n";
        case('M'): std::cout<<"The wolf hurt his leg.\n";
        case('E'): std::cout<<"The wolf lived happily everafter\n";
    }
    return 0;
}

int forLoop()
{
    for(int i=0; i< 10;i++)
    {
        std::cout<<"i = "<<i<<"\n";
    }
    return 0;
}

int whileLoop()
{
    int entry = 0;
    
    //with this while loop the condition is true
    //so the statements are executed
    while(entry <=5)
    {
        std::cout<<"incrementing entry = "<<entry<<"\n";
        entry++;
    }
    
    //with this while loop the condition is false
    //so the statements are not executed
    while(entry < 5)
    {
        std::cout<<"decrementing entry = "<<entry<<"\n";
        entry--;
    }
    
    return 0;
}

int doWhileLoop()
// Statements executed the first time through the loop
// BEFORE the condition is checked
{
    int count = 0;
    
    //This do..while loop will execute until count =5
    do
    {
        std::cout<<"Count = "<<count<<"\n";
        count++;
        }while(count < 5);
    
    
    int otherCount = 6;
    //This do..while loop will execute once. Even though
    //otherCount > 5
    do
    {
        std::cout<<"othercount = "<<otherCount<<"\n";
        otherCount++;
    }while(otherCount < 5);
    
    return 0;
}

int breakAndContinue()
{
    int a = 0;
    while(a < 5)
    {
        std::cout<<"a = "<<a<<"\n";
        a++;
        if(a == 3)
        break;
    }
    std::cout<<"The first statement after the first while loop\n\n";
    
    
    while(a < 15)
    {
        a++;
        if(a == 10)
        {
            std::cout<<"\tWhen a=10, go back to the top of the loop";
            std::cout<<"\n\tThis means a=10 is skipped.\n";
            continue;
        }
        std::cout<<"After continue a = "<<a<<"\n";
    }
    return 0;
}
