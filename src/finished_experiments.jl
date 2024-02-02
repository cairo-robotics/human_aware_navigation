env_name = "no_obstacles_50x50"
env_name = "small_obstacles_50x50"
env_name = "many_small_obstacles_50x50"
env_name = "L_shape_50x50"
env_name = "big_obstacle_50x50"

num_humans = 50
num_humans = 100
num_humans = 200

num_experiments = 100

data = ""
for (sudden_break,run_shield) in [ (false,false) ]
    data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
    datafile_name = "./RESULTS/"*string(num_experiments)*"_"*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
    data_dict = Dict("data"=>data);
    save(datafile_name, data_dict)
end
data_dict = ""
data = ""

s = load(datafile_name)
GC.gc();

data = ""
for (sudden_break,run_shield) in [ (true,false) ]
    data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
    datafile_name = "./RESULTS/"*string(num_experiments)*"_"*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
    data_dict = Dict("data"=>data);
    save(datafile_name, data_dict)
end

data_dict = ""
data = ""


#Shield
data = ""
for (sudden_break,run_shield) in [ (false,true) ]
    data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
    datafile_name = "./RESULTS/"*string(num_experiments)*"_"*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
    data_dict = Dict("data"=>data);
    save(datafile_name, data_dict)
end

#=
Ran On Server - 100 experiments

(SB - true; RS - false)
small - 200 (pane 5)

(SB - false; RS - false)
small - 200 (pane 1)
L - 200 (pane 4)
big - 200 (pane 3)


To be run on Server - 100 experiments

(SB - true; RS - false)
no - 200 ()
no - 100 ()

(SB - false; RS - false)
many - 200 ()
L - 200
big - 200
no - 200

=#



#=
Ran Locally - 100 experiments

(SB - true; RS - false)
small - 50  (pane 1)
L - 50 (pane 2)


To be run Locally - 100 experiments

(SB - true; RS - false)
no - 50 ()

(SB - false; RS - false)
many - 50  (pane 1)
big - 50 (pane 2)
L - 50  ()
small - 50 ()
no - 50 ()


=#


#=
Shield Expertiments

No - 50
Small - 50
Many - 50
L - 50
Big - 50

No - 100  (Omak - DONE)
Small - 100  (Omak - pane 6)
Many - 100
L - 100
Big - 100

No - 200
Small - 200
Many - 200
L - 200
Big - 200

=#

num_experiments = 100

#DOMAIN 1
env_name = "no_obstacles_50x50"
#DOMAIN 2
env_name = "small_obstacles_50x50"
#DOMAIN 3
env_name = "many_small_obstacles_50x50"
#DOMAIN 4
env_name = "L_shape_50x50"
#DOMAIN 5
env_name = "big_obstacle_50x50"

num_humans = 50
num_humans = 100
num_humans = 200


#NO SB
sudden_break = false
run_shield = false

#SB
sudden_break = true
run_shield = false

#SHIELD
sudden_break = false
run_shield = true

prefix = "./RESULTS/"
prefix = "/media/himanshu_storage/RESULTS_100_expts_Jan29/"

fn = prefix*string(num_experiments)*"_"*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
s = load(fn);
data = s["data"];

data.environment_name
data.num_humans
data.num_experiments
data.sudden_break_flag
data.run_shield_flag

#=
Debugging Tool

for (i,j) in enumerate(data.esp_hjb)
   if j.vehicle_reached_goal == false
       println(j.time_taken, " ", j.vehicle_ran_into_obstacle, " ",j.vehicle_ran_into_boundary_wall ," ",i)
   end
end

for (i,j) in enumerate(data.baseline)
   if j.vehicle_reached_goal == false
       println(j.time_taken, " ", j.vehicle_ran_into_obstacle, " ",j.vehicle_ran_into_boundary_wall ," ",i)
   end
end
=#


#=
Few File Locations

/home/himanshu/Desktop/FromOMAK/no_obstacles_50x50_humans_50_suddenbreak_true_runshield_false.jld2

=#
