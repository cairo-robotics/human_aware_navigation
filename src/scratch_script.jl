extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide);
pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    max_trials=20,tree_in_info=true);#,default_action=default_es_pomdp_action)
pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
ti = 2.0
b = output.b_root[ti]
next_pomdp_action, info = action_info(pomdp_planner, b)
@profiler action_info(pomdp_planner, b)
