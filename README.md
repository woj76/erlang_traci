# erlang_traci -- SUMO TraCI interface implementation in Erlang

OVERVIEW

This is currently work in progress and only a small fraction of the TraCI
interface is implemented. For more information about TraCI and SUMO see
https://sumo.dlr.de/docs/TraCI.html.

INSTALLATION

Compile the traci.erl file with the Erlang compiler and make sure the
traci.beam file is reachable by the Erlang environment. In the simplest
case place the traci.erl file in your current directory and say:

	c(traci).

from the Erlang shell.

USE

Currently there is no detailed (any actually) documentation, we rely on
you knowing both TraCI and Erlang, check the SUMO pages for examples in
Python and do code matching if in doubt. A simple sequence of calls to
get the simulation going from Erlang is this:

	c(traci). % or l(traci).
	traci:open(8813). % Choose your SUMO TCP port number
	traci:set_order(1). % Optional
	traci:simulation_step(). % Next simulation step
	traci:close().

Before that, you have to start SUMO manually outside of the Erlang
environment, starting the SUMO process from within Erlang (like it is
possible in Python) is currently not supported.

AUTHOR / LICENSE / DISCLAIMER

The author of this project and code is Wojciech Mostowski
<wojciech.mostowski@gmail.com>. The code is distributed under the terms
of GNU General Public License Version 3. The author of this project takes
no responsibility nor is liable for any damage or loss caused by the use
of the project code. Furthermore, no suitability for any particular
purpose is promised. Otherwise see the GPL license conditions for details
on the lack of any warranty. Having said that, you can always drop the
author a line if/when you find the project useful or when you want to
contribute to this project.

