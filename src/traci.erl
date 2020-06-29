%%    erlang_traci -- SUMO TraCI interface implementation in Erlang
%%    Copyright (C) 2020 Wojciech Mostowski <wojciech.mostowski@gmail.com>
%%
%%    This program is free software: you can redistribute it and/or modify
%%    it under the terms of the GNU General Public License as published by
%%    the Free Software Foundation, either version 3 of the License, or
%%    (at your option) any later version.
%%
%%    This program is distributed in the hope that it will be useful,
%%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%%    GNU General Public License for more details.
%%
%%    You should have received a copy of the GNU General Public License
%%    along with this program.  If not, see <https://www.gnu.org/licenses/>.

-module(traci).

-export([
	open/1, open/2, close/0,
	set_order/1, simulation_step/0,
	add_vehicle/3,
	vehicle_get_idlist/0, vehicle_get_roadid/1, vehicle_get_typeid/1,
	vehicle_get_laneid/1, vehicle_get_routeid/1, vehicle_get_route_index/1,
	vehicle_get_route/1, vehicle_get_distance/1, vehicle_get_lane_position/1,
	vehicle_get_position/1, vehicle_get_position3d/1,
	light_get_idlist/0, light_get_state/1, light_get_lanes/1, light_get_links/1,
	lane_get_edgeid/1, lane_get_length/1
]).

%% TraCI constants

-define(TRACI_CMD_SIMULATION_STEP, 2).
-define(TRACI_CMD_SET_ORDER, 3).
-define(TRACI_CMD_SET_VEHICLE_VARIABLE, 16#C4).

-define(TRACI_CMD_GET_VEHICLE_VARIABLE, 16#A4).
-define(TRACI_RPL_GET_VEHICLE_VARIABLE, 16#B4).

-define(TRACI_CMD_GET_LIGHT_VARIABLE, 16#A2).
-define(TRACI_RPL_GET_LIGHT_VARIABLE, 16#B2).

-define(TRACI_CMD_GET_LANE_VARIABLE, 16#A3).
-define(TRACI_RPL_GET_LANE_VARIABLE, 16#B3).

-define(TRACI_RPL_SUB_VEHICLE_VARIABLE, 16#E4).
-define(TRACI_RPL_SUB_SIMULATION_VARIABLE, 16#EB).

-define(TRACI_STATE_ADD, 16#85).

-define(TRACI_VAR_ROADID, 16#50).
-define(TRACI_VAR_LANEID, 16#51).
-define(TRACI_VAR_ROUTEID, 16#53).
-define(TRACI_VAR_ROUTE_INDEX, 16#69).
-define(TRACI_VAR_EDGES, 16#54).
-define(TRACI_VAR_TYPEID, 16#4F).
-define(TRACI_VAR_IDLIST, 16#00).
-define(TRACI_VAR_LSTATE, 16#20).
-define(TRACI_VAR_LANES, 16#26).
-define(TRACI_VAR_LINKS, 16#27).
-define(TRACI_VAR_EDGEID, 16#31).
-define(TRACI_VAR_LENGTH, 16#44).
-define(TRACI_VAR_LANE_POS, 16#56).
-define(TRACI_VAR_DISTANCE, 16#84).
-define(TRACI_VAR_POSITION, 16#42).
-define(TRACI_VAR_POSITION3D, 16#39).
-define(TRACI_VAR_SPEED, 16#40).
-define(TRACI_VAR_ANGLE, 16#43).
-define(TRACI_VAR_WIDTH, 16#4D).
-define(TRACI_VAR_HEIGHT, 16#BC).
-define(TRACI_VAR_SIGNAL_STATES, 16#5B).

-define(TRACI_TYPE_COMPOUND, 16#0F).
-define(TRACI_TYPE_STRING, 16#0C).
-define(TRACI_TYPE_STRING_LIST, 16#0E).
-define(TRACI_TYPE_INTEGER, 16#09).
-define(TRACI_TYPE_DOUBLE, 16#0B).
-define(TRACI_TYPE_UBYTE, 16#07).
-define(TRACI_TYPE_BYTE, 16#08).
-define(TRACI_TYPE_POSITION2D, 16#01).
-define(TRACI_TYPE_POSITION3D, 16#03).

%% Private part

command(CmdId, <<CmdData/binary>>) ->
	Sock = erlang:get(sumo_socket),
	Len = 2 + byte_size(CmdData),
	case gen_tcp:send(Sock, <<(4+Len):4/integer-unit:8, Len, CmdId, CmdData/binary>>) of
		ok ->
			case gen_tcp:recv(Sock, 4+1+1+1+4) of
				{error, _} -> closed;
				Received ->
					{ok, << TotalLength:4/integer-unit:8, _RL, CmdId, StatusCode, StatusMessageLength:4/integer-unit:8 >> } = Received,
					Status = case StatusMessageLength of
						0 -> ok;
						_ -> {ok, StatusMessage} = gen_tcp:recv(Sock, StatusMessageLength), {error, StatusCode, binary_to_list(StatusMessage)}
					end,
					RemainingLength = TotalLength - (4+1+1+1+4) - StatusMessageLength,
					{ok, ResultData} = case RemainingLength of
						0 -> {ok, empty};
						_ -> gen_tcp:recv(Sock, RemainingLength)
					end,
					%	io:format("Result data ~p ~n", [ResultData]),
					case Status of
						ok -> case ResultData of
							empty -> ok;
							_ -> ResultData
						end;
						_ -> Status
					end
			end;
	        _ -> closed
        end.

var_id(?TRACI_VAR_ROADID) -> var_road_id;
var_id(?TRACI_VAR_LANEID) -> var_lane_id;
var_id(?TRACI_VAR_DISTANCE) -> var_distance;
var_id(?TRACI_VAR_POSITION) -> var_position;
var_id(?TRACI_VAR_POSITION3D) -> var_position3d;
var_id(?TRACI_VAR_IDLIST) -> var_id_list;
var_id(?TRACI_VAR_TYPEID) -> var_type_id;
var_id(?TRACI_VAR_ROUTEID) -> var_route_id;
var_id(?TRACI_VAR_ROUTE_INDEX) -> var_route_index;
var_id(?TRACI_VAR_EDGES) -> var_edges;
var_id(?TRACI_VAR_EDGEID) -> var_edge_id;
var_id(?TRACI_VAR_LSTATE) -> var_light_state;
var_id(?TRACI_VAR_LANES) -> var_lanes;
var_id(?TRACI_VAR_LINKS) -> var_links;
var_id(?TRACI_VAR_LANE_POS) -> var_lane_position;
var_id(?TRACI_VAR_SPEED) -> var_speed;
var_id(?TRACI_VAR_ANGLE) -> var_angle;
var_id(?TRACI_VAR_LENGTH) -> var_length;
var_id(?TRACI_VAR_WIDTH) -> var_width;
var_id(?TRACI_VAR_HEIGHT) -> var_height;
var_id(?TRACI_VAR_SIGNAL_STATES) -> var_signals;
var_id(Id) -> {var_id, Id}.

var_status(0) -> ok;
var_status(16#FF) -> error;
var_status(Status) -> {var_status, Status}.

reply_id(?TRACI_RPL_SUB_VEHICLE_VARIABLE) -> reply_subscribe_vehicle_var;
reply_id(?TRACI_RPL_SUB_SIMULATION_VARIABLE) -> reply_subscribe_simulation_var;
reply_id(?TRACI_RPL_GET_VEHICLE_VARIABLE) -> reply_get_vehicle_var;
reply_id(?TRACI_RPL_GET_LIGHT_VARIABLE) -> reply_get_light_var;
reply_id(?TRACI_RPL_GET_LANE_VARIABLE) -> reply_get_lane_var;
reply_id(Id) -> {reply_id, Id}.

format_integer(Number) ->
	<< ?TRACI_TYPE_INTEGER, Number:4/signed-integer-unit:8 >>.

format_str(Str) ->
	StrLength = length(Str), StrData = list_to_binary(Str),
	<< StrLength:4/integer-unit:8, StrData/binary >>.

format_string(Str) ->
	B = format_str(Str), << ?TRACI_TYPE_STRING, B/binary >>.

format_compound(List) ->
        ListLength = length(List),
	ListData = list_to_binary(List),
	<< ?TRACI_TYPE_COMPOUND, ListLength:4/integer-unit:8, ListData/binary >>.

unpack_binary(0, Result, Data, _) -> {Result, Data};
unpack_binary(L, Result, Data, F) ->
	{Item, Rest} = F(Data),
        unpack_binary(L-1, Result ++ [Item], Rest, F).

deformat_string(L, Result, Data) ->
	unpack_binary(L, Result, Data,
		fun(X) -> << C, R/binary >> = X, {C, R} end).

deformat_string_list(L, Result, Data) ->
	unpack_binary(L, Result, Data,
		fun (X) ->
			<< SL:4/unsigned-integer-unit:8, R/binary >> = X,
			deformat_string(SL, [], R)
		end).

deformat_reply_vars(L, Result, Data) ->
	unpack_binary(L, Result, Data,
		fun(X) ->
			<< VarId, VarStatus, D/binary >> = X,
			{Var, R} = deformat(D),
			{{var_id(VarId), var_status(VarStatus), Var}, R}
		end
	).

deformat_reply(<<0, _RL:4/unsigned-integer-unit:8, RespId, ObjectIDLen:4/integer-unit:8, Data/binary>>) ->
	{ObjectId, <<NumVars, Rest0/binary>>} =
		deformat_string(ObjectIDLen, "", Data),
	{Vars, Rest} = deformat_reply_vars(NumVars, [], Rest0),
	{ {reply_id(RespId), {object_id, ObjectId}, {variables, Vars} },
	  Rest };

deformat_reply(Data) -> deformat_reply(<<0, 0, 0, 0, Data/binary>>).

deformat_replies(L, Result, Data) ->
	unpack_binary(L, Result, Data, fun deformat_reply/1).

deformat(<< ?TRACI_TYPE_STRING, L:4/unsigned-integer-unit:8, StrData/binary>>) ->
	deformat_string(L, "", StrData);
deformat(<< ?TRACI_TYPE_STRING_LIST, L:4/unsigned-integer-unit:8, StrsData/binary>>) ->
	deformat_string_list(L, [], StrsData);
deformat(<< ?TRACI_TYPE_COMPOUND, L:4/unsigned-integer-unit:8, Data/binary>>) ->
	{L, Data};
%	traci_deformat_compound(L, [], Data);
deformat(<< ?TRACI_TYPE_INTEGER, I:4/integer-unit:8, Rest/binary>>) ->
	{I, Rest};
deformat(<< ?TRACI_TYPE_DOUBLE, D/float, Rest/binary>>) ->
	{D, Rest};
deformat(<< ?TRACI_TYPE_POSITION2D, X/float, Y/float, Rest/binary>>) ->
	{ {X, Y}, Rest };
deformat(<< ?TRACI_TYPE_POSITION3D,
	X/float, Y/float, Z/float, Rest/binary>>) -> { {X, Y, Z}, Rest};
deformat(B) -> {tbd, B}.

get_variable(CmdCode, RplCode, ItemId, VariableId) ->
	S = format_str(ItemId),
	case command(CmdCode, << VariableId, S/binary >>) of
		closed -> closed;
		Data ->
			<< FL, Result0/binary>> = Data,
			<< RplCode, VariableId, Result1/binary >> =
				case FL of
					0 -> <<_:4/unsigned-integer-unit:8, Rest/binary>> = Result0, Rest;
	        			_ -> Result0
				end,
			{ItemId, Result3} = deformat(<< ?TRACI_TYPE_STRING, Result1/binary>>),
			Result3
	end.

get_variable_deformat(CodeGet, CodeReply, ItemId, VariableId) ->
	case get_variable(CodeGet, CodeReply, ItemId, VariableId) of
		closed -> closed;
		Data -> {Result, <<>>} = deformat(Data), Result
	end.

deformat_link(L, Result, Data) -> unpack_binary(L, Result, Data, fun deformat/1).

%% Public part

% The basics / first steps:

open(Host, PortNum) ->
	{ok, Sock} = gen_tcp:connect(Host, PortNum,
				[binary,
				 {packet, 0}, % {packet, 4}???
				 {active, false}]),
	erlang:put(sumo_socket, Sock),
	ok.

open(PortNum) -> open("localhost", PortNum).

close() -> ok = gen_tcp:close(erlang:get(sumo_socket)).

set_order(Order) -> command(?TRACI_CMD_SET_ORDER, <<Order:4/integer-unit:8>>).

simulation_step() ->
	case command(?TRACI_CMD_SIMULATION_STEP, <<0/float>>) of
		closed -> closed;
	        SR ->
			<<NumReplies:4/unsigned-integer-unit:8, Replies/binary>> = SR,
			{R, <<>>} = deformat_replies(NumReplies, [], Replies),
			R
	end.

% Change simulation state interface:

add_vehicle(VehicleId, RouteId, VehicleTypeId) ->
   	S = format_str(VehicleId),
   	B = format_compound([
   		format_string(RouteId),
   		format_string(VehicleTypeId),
   		format_string("now"), % depart time
   		format_string("first"), % depart lane
   		format_string("base"), % depart position
   		format_string("0"), % depart speed
   		format_string("current"), % arrival lane
   		format_string("max"), % arrival position
   		format_string("current"), % arrival speed
   		format_string(""), % from taz
   		format_string(""), % to taz
   		format_string(""), % line # (public transport)
   		format_integer(0), % person capacity
   		format_integer(0) % # person
   	]),
   	command(?TRACI_CMD_SET_VEHICLE_VARIABLE, << ?TRACI_STATE_ADD, S/binary, B/binary >>).

% TraCI vehicle get interface:

vehicle_get_idlist() ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		"0", ?TRACI_VAR_IDLIST).

vehicle_get_roadid(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_ROADID).

vehicle_get_typeid(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_TYPEID).

vehicle_get_laneid(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_LANEID).

vehicle_get_routeid(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_ROUTEID).

vehicle_get_route_index(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_ROUTE_INDEX).

vehicle_get_route(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_EDGES).

vehicle_get_distance(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_DISTANCE).

vehicle_get_lane_position(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_LANE_POS).

vehicle_get_position(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_POSITION).

vehicle_get_position3d(VehicleId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_VEHICLE_VARIABLE, ?TRACI_RPL_GET_VEHICLE_VARIABLE,
		VehicleId, ?TRACI_VAR_POSITION3D).

% TraCI traffic light get interface:

light_get_idlist() ->
	get_variable_deformat(
		?TRACI_CMD_GET_LIGHT_VARIABLE, ?TRACI_RPL_GET_LIGHT_VARIABLE,
		"0", ?TRACI_VAR_IDLIST).

light_get_state(LightId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_LIGHT_VARIABLE, ?TRACI_RPL_GET_LIGHT_VARIABLE,
		LightId, ?TRACI_VAR_LSTATE).

light_get_lanes(LightId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_LIGHT_VARIABLE, ?TRACI_RPL_GET_LIGHT_VARIABLE,
		LightId, ?TRACI_VAR_LANES).

light_get_links(LightId) ->
	case get_variable(
			?TRACI_CMD_GET_LIGHT_VARIABLE, ?TRACI_RPL_GET_LIGHT_VARIABLE,
			LightId, ?TRACI_VAR_LINKS) of
		closed -> closed;
		Data ->
			{_, Rest1} = deformat(Data),
			{NumSignals, Rest} = deformat(Rest1), % integer
			{Result, <<>>} = unpack_binary(NumSignals, [], Rest,
	  			fun(D) -> {N, R} = deformat(D), deformat_link(N, [], R) end),
			Result
	end.

% TraCI lane get interface:

lane_get_edgeid(LaneId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_LANE_VARIABLE, ?TRACI_RPL_GET_LANE_VARIABLE,
		LaneId, ?TRACI_VAR_EDGEID).

lane_get_length(LaneId) ->
	get_variable_deformat(
		?TRACI_CMD_GET_LANE_VARIABLE, ?TRACI_RPL_GET_LANE_VARIABLE,
		LaneId, ?TRACI_VAR_LENGTH).

