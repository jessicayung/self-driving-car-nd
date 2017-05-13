# Reflection

## Effect of P, I, D components 
 
### P: 

* Helps steer in proportion to the crosstrack error
* Accounts for present values of error

Model value: P = 0.1

<table>
	<th>Param state</th><th>Description</th>
	<tr>
		<td>Higher P,
		P = 0.5</td>
		<td>Sharp turns, did well during the initial straight road. Might start weaving like mad if it goes off. Initially went off track after the bridge.</td>
	</tr>
	<tr>
		<td>Lower P, P = 0.001</td>
		<td>Immediately went off the track during the initial straight road portion. Took too long to adjust. (Was hardly changing its steering direction.)</td>
	</tr>
</table>


### I: Integral

* Helps steer more when there is sustained error to counter the systematic bias we have from e.g. misaligned wheels.
* Accounts for past values of the error (integral accumulates over time, may steer if accumulated error is large even when present error is not strong enough)

Expected optimal value is low because we don't expect wheels in the simulator to be misaligned much.

Model value: I = 0.005
<table>
	<th>Param state</th><th>Description</th>
	<tr>
		<td>Higher I, I = 0.2</td>
		<td>Over-accounts for bias and immediately veers off.</td>
	</tr>
	<tr>
		<td>Lower I, I = 0</td>
		<td>Car position seems slightly shifted, car tyres sometimes touch the edge of the lane (e.g. red and white portions).</td>
	</tr>
</table>

### D: 

* When the car has turned enough to reduce CTE, it counter-steers to avoid overshooting.
* Accounts for possible future trends of the error because it's based on current rate of change of error

Model value: D = 4.0

<table>
	<th>Param state</th><th>Description</th>
	<tr>
		<td>Higher D, D = 5.0</td>
		<td>Counter-steers too much.</td>
	</tr>
	<tr>
		<td>Lower D: D = 2.0</td>
		<td>It does not counter-steer. (Can see in the second adjustment when the car is too far off to the right on the initial straight segment and goes off the track instead of steering left.)</td>
	</tr>
</table>

### How the final hyperparameters were chosen

The final hyperparameters (0.1, 0.005, 4.0) were chosen manually.

### References:
* [PID Controller (Wikipedia)](https://en.wikipedia.org/wiki/PID_controller)
