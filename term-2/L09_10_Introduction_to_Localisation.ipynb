{
 "cells": [
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "### Introduction to Localisation\n",
    "Localisation allows a car to know precisely where it is.\n",
    "* Answers Q: 'Where is our car in a given map with an accuracy of 3-10cm?'\n",
    "    * GPS is not precise enough (accuracy of the width of a lane, 1-3m. Sometimes as broad as 10-15m.)\n",
    "    * Robot takes info about environment and compares it to what it knows about the real world (a map).\n",
    "    * Intuition: Reducing uncertainty about location using information e.g. seeing the Eiffel Tower.\n",
    "* Use onboard sensors (lidar, radar) to measure distance to static obstacles and bearings in the local coordinate system of our car. Some obstacles that are observed by onboard sensors are also on the map (which has a global coordinate system.)\n",
    "    * Need to match observations with map information (transform both coordinates) with high precision. \n",
    "(9.1, 9.2)    \n",
    "\n",
    "![](images/9.1.png)\n",
    "\n",
    "![](images/9.2.png)\n",
    "\n",
    "Lesson Outline\n",
    "* What is Localisation?\n",
    "* Bayesian Filters: 1D Example in C++\n",
    "* Motion Models\n",
    "* 2D Particle Filter in C++"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "# Localisation\n",
    "\n",
    "\n",
    "## What is localisation?\n",
    "\n",
    "![](images/10.1.png)\n",
    "\n",
    "* Measurement of door transforms our belief function: increased belief of locations next to the door, decreased belief of locations not next to the door.\n",
    "* Third dist: Assume robot moves to the right by a certain distance. Shift the belief to the right by a certain dist (but peaks are flatter).\n",
    "* Fourth dist: Sees door again after a small amout of rightward movement.\n",
    "\n",
    "## Uniform Dist code"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": [
    "#  Modify your code to create probability vectors, p, of arbitrary \n",
    "#  size, n. Use n=5 to verify that your new solution matches \n",
    "#  the previous one.\n",
    "\n",
    "n=5\n",
    "p=[1.0/n for i in range(n)]\n",
    "\n",
    "print(p)"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "### Updating probabilities after sensing\n",
    "\n",
    "![](images/10.2.png)\n",
    "* Multiply belief it's in a red cell by e.g. 0.6, multiply belief it's in a green cell by 0.2.\n",
    "* Normalise by sum of values to make posterior a valid probability distribution.\n"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "### Program localisation function in Python"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": [
    "# Localisation function in Python\n",
    "\n",
    "p=[0.2, 0.2, 0.2, 0.2, 0.2]\n",
    "world=['green', 'red', 'red', 'green', 'green']\n",
    "measurements = ['red', 'green']\n",
    "pHit = 0.6\n",
    "pMiss = 0.2\n",
    "\n",
    "def sense(p, Z):\n",
    "    q=[]\n",
    "    for i in range(len(p)):\n",
    "        hit = (Z == world[i])\n",
    "        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))\n",
    "    s = sum(q)\n",
    "    for i in range(len(q)):\n",
    "        q[i] = q[i] / s\n",
    "    return q\n",
    "\n",
    "for Z in measurements:\n",
    "    p = sense(p, Z)\n",
    "print(p)\n"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "### Program exact and inaccurate motion in Python\n",
    "* Exact motion case\n"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": [
    "#Program a function that returns a new distribution \n",
    "#q, shifted to the right by U units. If U=0, q should \n",
    "#be the same as p.\n",
    "\n",
    "p=[0, 1, 0, 0, 0]\n",
    "world=['green', 'red', 'red', 'green', 'green']\n",
    "measurements = ['red', 'green']\n",
    "pHit = 0.6\n",
    "pMiss = 0.2\n",
    "\n",
    "def sense(p, Z):\n",
    "    q=[]\n",
    "    for i in range(len(p)):\n",
    "        hit = (Z == world[i])\n",
    "        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))\n",
    "    s = sum(q)\n",
    "    for i in range(len(q)):\n",
    "        q[i] = q[i] / s\n",
    "    return q\n",
    "\n",
    "def move(p, U):\n",
    "    #\n",
    "    #ADD CODE HERE\n",
    "    #\n",
    "    U %= len(p)\n",
    "    q = p[-U:] + p[:-U]\n",
    "    // or\n",
    "    // for i in range(len(p)):\n",
    "    //     q.append(p[(i-U) % len(p)])\n",
    "    return q\n",
    "\n",
    "print move(p, 1)\n"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "* Inaccurate motion case\n",
    "![](images/10.3.png)\n"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": [
    "#Modify the move function to accommodate the added \n",
    "#probabilities of overshooting or undershooting \n",
    "#the intended destination.\n",
    "\n",
    "p=[0, 1, 0, 0, 0]\n",
    "world=['green', 'red', 'red', 'green', 'green']\n",
    "measurements = ['red', 'green']\n",
    "pHit = 0.6\n",
    "pMiss = 0.2\n",
    "pExact = 0.8\n",
    "pOvershoot = 0.1\n",
    "pUndershoot = 0.1\n",
    "\n",
    "def sense(p, Z):\n",
    "    q=[]\n",
    "    for i in range(len(p)):\n",
    "        hit = (Z == world[i])\n",
    "        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))\n",
    "    s = sum(q)\n",
    "    for i in range(len(q)):\n",
    "        q[i] = q[i] / s\n",
    "    return q\n",
    "\n",
    "def move(p, U):\n",
    "    q = []\n",
    "    for i in range(len(p)):\n",
    "        q.append(p[(i-U)%len(p)]*pExact + p[(i-U-1)%len(p)]*pUndershoot + p[(i-U+1)%len(p)]*pOvershoot)\n",
    "    return q\n",
    "    \n",
    "\n",
    "print move(p, 1)\n"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "### Limit distribution\n",
    "* Uniform dist: maximal uncertainty, min information. every time we move we lose information (uncertainty increases).\n",
    "* Balance property: P(X) breakdown must hold in the limit as when things don't move any more. (?)\n",
    "\n",
    "![](images/10.4.png)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": [
    "#write code that moves 1000 times and then prints the \n",
    "#resulting probability distribution.\n",
    "# p approx equals [0.2, 0.2, 0.2, 0.2, 0.2]\n",
    "\n",
    "p=[0, 1, 0, 0, 0]\n",
    "world=['green', 'red', 'red', 'green', 'green']\n",
    "measurements = ['red', 'green']\n",
    "pHit = 0.6\n",
    "pMiss = 0.2\n",
    "pExact = 0.8\n",
    "pOvershoot = 0.1\n",
    "pUndershoot = 0.1\n",
    "\n",
    "def sense(p, Z):\n",
    "    q=[]\n",
    "    for i in range(len(p)):\n",
    "        hit = (Z == world[i])\n",
    "        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))\n",
    "    s = sum(q)\n",
    "    for i in range(len(q)):\n",
    "        q[i] = q[i] / s\n",
    "    return q\n",
    "\n",
    "def move(p, U):\n",
    "    q = []\n",
    "    for i in range(len(p)):\n",
    "        s = pExact * p[(i-U) % len(p)]\n",
    "        s = s + pOvershoot * p[(i-U-1) % len(p)]\n",
    "        s = s + pUndershoot * p[(i-U+1) % len(p)]\n",
    "        q.append(s)\n",
    "    return q\n",
    "#\n",
    "# ADD CODE HERE\n",
    "#\n",
    "for i in range(1000):\n",
    "    p = move(p, 1)\n",
    "print p\n"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "### Localisation is cycling through 'Sense and Move'\n",
    "\n",
    "![](images/10.5.png)\n",
    "\n",
    "* Entropy: Measure of information Expected log-likelihood. \n",
    "    * Update step (move) makes entropy go down (lose info), measurement step (sense) makes entropy go up (gain info).\n",
    "\n"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {
    "collapsed": true
   },
   "outputs": [],
   "source": [
    "#Given the list motions=[1,1] which means the robot \n",
    "#moves right and then right again, compute the posterior \n",
    "#distribution if the robot first senses red, then moves \n",
    "#right one, then senses green, then moves right again, \n",
    "#starting with a uniform prior distribution.\n",
    "\n",
    "p=[0.2, 0.2, 0.2, 0.2, 0.2]\n",
    "world=['green', 'red', 'red', 'green', 'green']\n",
    "measurements = ['red', 'green']\n",
    "motions = [1,1]\n",
    "pHit = 0.6\n",
    "pMiss = 0.2\n",
    "pExact = 0.8\n",
    "pOvershoot = 0.1\n",
    "pUndershoot = 0.1\n",
    "\n",
    "def sense(p, Z):\n",
    "    q=[]\n",
    "    for i in range(len(p)):\n",
    "        hit = (Z == world[i])\n",
    "        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))\n",
    "    s = sum(q)\n",
    "    for i in range(len(q)):\n",
    "        q[i] = q[i] / s\n",
    "    return q\n",
    "\n",
    "def move(p, U):\n",
    "    q = []\n",
    "    for i in range(len(p)):\n",
    "        s = pExact * p[(i-U) % len(p)]\n",
    "        s = s + pOvershoot * p[(i-U-1) % len(p)]\n",
    "        s = s + pUndershoot * p[(i-U+1) % len(p)]\n",
    "        q.append(s)\n",
    "    return q\n",
    "#\n",
    "# ADD CODE HERE\n",
    "#\n",
    "for i in range(len(motions)):\n",
    "    p = sense(p, measurements[i])\n",
    "    p = move(p, motions[i])\n",
    "print p         \n"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "### Summary\n",
    "![](images/10.6.png)\n",
    "![](images/10.7.png)\n",
    "\n",
    "* Move: Convolution: Reverse engineer the situation.\n",
    "\n",
    "### Αppendix\n",
    "#### Formalising localisation\n",
    "\n",
    "![](images/10.8.png)\n",
    "![](images/10.9.png)\n",
    "* P(Z) technically probability of observing dist without location information.\n",
    "* Alpha is normaliser\n"
   ]
  }
 ],
 "metadata": {
  "kernelspec": {
   "display_name": "Python 3",
   "language": "python",
   "name": "python3"
  },
  "language_info": {
   "codemirror_mode": {
    "name": "ipython",
    "version": 3
   },
   "file_extension": ".py",
   "mimetype": "text/x-python",
   "name": "python",
   "nbconvert_exporter": "python",
   "pygments_lexer": "ipython3",
   "version": "3.6.0"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 2
}
