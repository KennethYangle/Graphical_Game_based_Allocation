import numpy as np
from scipy.optimize import linprog
from math import floor
from itertools import product
import collections
import time
# import matplotlib.pyplot as plt # Plotting disabled for text output

# --- Data Structures ---
Worker = collections.namedtuple("Worker", ["id", "original_interceptor_id", "pos"])
Task = collections.namedtuple("Task", ["id", "original_intruder_id", "pos"])

# --- Configuration ---
R_PANO = 6.0
R_TELE = 7.0
D_PAYOFF = 15.0

# --- Algorithm 1: Construct Worker/Task Lists ---
def construct_list(interceptors_pos, intruders_pos):
    I_count = len(interceptors_pos)
    H_count = len(intruders_pos)
    workers = []
    tasks = []
    if I_count == 0 or H_count == 0: return [], [], I_count, H_count
    target_n = max(I_count, H_count)

    if I_count == H_count:
        for i in range(I_count):
            workers.append(Worker(id=i, original_interceptor_id=i, pos=interceptors_pos[i]))
            tasks.append(Task(id=i, original_intruder_id=i, pos=intruders_pos[i]))
    elif I_count < H_count:
        t = floor((H_count - 1) / I_count) + 1
        worker_idx = 0
        for i in range(I_count):
            for k in range(t):
                 if worker_idx < target_n:
                    workers.append(Worker(id=worker_idx, original_interceptor_id=i, pos=interceptors_pos[i]))
                    worker_idx += 1
        for j in range(H_count):
             if j < target_n: tasks.append(Task(id=j, original_intruder_id=j, pos=intruders_pos[j]))
        n = len(tasks) # Should be target_n
        # Pad workers only if absolutely needed (less likely with target_n logic)
        while len(workers) < n:
            pad_id = workers[-1].original_interceptor_id
            pad_pos = interceptors_pos[pad_id] # Use original interceptor pos
            workers.append(Worker(id=len(workers), original_interceptor_id=pad_id, pos=pad_pos))
            print(f"Padding worker list: Added worker {len(workers)-1}")
        workers = workers[:n] # Ensure exactly n workers

        # Pad tasks if H_count < target_n (less likely here)
        while len(tasks) < n:
             pad_id = tasks[-1].original_intruder_id
             pad_pos = intruders_pos[pad_id]
             tasks.append(Task(id=len(tasks), original_intruder_id=pad_id, pos=pad_pos))
             print(f"Padding task list: Added task {len(tasks)-1}")
        tasks = tasks[:n]

    else: # I > H
        t = floor((I_count - 1) / H_count) + 1
        task_idx = 0
        for j in range(H_count):
            for k in range(t):
                if task_idx < target_n:
                    tasks.append(Task(id=task_idx, original_intruder_id=j, pos=intruders_pos[j]))
                    task_idx += 1
        for i in range(I_count):
             if i < target_n: workers.append(Worker(id=i, original_interceptor_id=i, pos=interceptors_pos[i]))
        n = len(workers)
        # Pad tasks if necessary
        while len(tasks) < n:
            pad_id = tasks[-1].original_intruder_id
            pad_pos = intruders_pos[pad_id]
            tasks.append(Task(id=len(tasks), original_intruder_id=pad_id, pos=pad_pos))
            print(f"Padding task list: Added task {len(tasks)-1}")
        tasks = tasks[:n]
        # Pad workers if I_count < target_n
        while len(workers) < n:
            pad_id = workers[-1].original_interceptor_id
            pad_pos = interceptors_pos[pad_id]
            workers.append(Worker(id=len(workers), original_interceptor_id=pad_id, pos=pad_pos))
            print(f"Padding worker list: Added worker {len(workers)-1}")
        workers = workers[:n]

    print(f"\n[步骤一 S14 结果] ConstructList: Original I={I_count}, H={H_count}.")
    print(f"  最终工人列表 W (大小 n={len(workers)}):")
    for w in workers: print(f"    {w}")
    print(f"  最终任务列表 T (大小 n={len(tasks)}):")
    for t in tasks: print(f"    {t}")

    if len(workers) != len(tasks):
         print(f"错误: 工人列表和任务列表长度不匹配 |W|={len(workers)}, |T|={len(tasks)}")
         # Fallback to min length, though ideally should match max(I, H)
         n = min(len(workers), len(tasks)); workers = workers[:n]; tasks = tasks[:n]

    return workers, tasks, I_count, H_count

# --- Graph and Neighborhood Construction ---
def build_graph_and_neighborhoods(workers, tasks):
    n = len(workers)
    adj = collections.defaultdict(list)
    worker_neighbors_nwl = {}
    worker_targets_ntl = {}
    print("\n[步骤二 S21 预备] 计算邻居和可达目标:")
    print("  计算工人间距离:")
    worker_map = {w.id: w for w in workers}
    for i in range(n):
        for j in range(i, n):
            dist = np.linalg.norm(worker_map[i].pos - worker_map[j].pos)
            print(f"    Dist(W{i}, W{j}) = {dist:.2f}")
            if dist <= R_PANO and i != j : # Allow 0 distance for same interceptor but don't add self edge yet
                adj[i].append(j)
                adj[j].append(i)

    print("  计算工人到任务距离:")
    task_map = {t.id: t for t in tasks}
    for i in range(n):
        worker_id = i # Worker ID matches index
        # NWL: Neighbors based on graph (including self)
        nwl = sorted(list(set(adj[worker_id] + [worker_id])))
        worker_neighbors_nwl[worker_id] = nwl

        # NTL: Tasks reachable by this worker
        ntl = []
        print(f"    对于 W{worker_id} (来自 I{worker_map[worker_id].original_interceptor_id}):")
        for task_id in range(len(tasks)):
            dist = np.linalg.norm(worker_map[worker_id].pos - task_map[task_id].pos)
            print(f"      Dist(W{worker_id}, T{task_id}) = {dist:.2f}", end="")
            # Simplified FOV check (only range R_TELE)
            if dist <= R_TELE:
                ntl.append(task_id)
                print(" -> 可达")
            else:
                print(" -> 不可达")

        if not ntl:
             if tasks: ntl.append(tasks[0].id) # Fallback task
             else: ntl.append(-1) # No tasks available
             print(f"      W{worker_id} 没有可达目标，分配默认 T0 或 -1")

        worker_targets_ntl[worker_id] = sorted(list(set(ntl)))

    print("\n[步骤二 S21 结果] 邻居集 (NWL):")
    for wid, nwl in worker_neighbors_nwl.items(): print(f"  NWL(W{wid}) = {nwl}")
    print("[步骤二 S21 结果] 可攻击目标集 (NTL):")
    for wid, ntl in worker_targets_ntl.items(): print(f"  NTL(W{wid}) = {ntl}")

    return worker_neighbors_nwl, worker_targets_ntl

# --- Payoff Calculation (Helper) ---
def calculate_payoff(worker_id, action_task_id, joint_action_si_tuple,
                       worker_neighbors_nwl, worker_targets_ntl,
                       workers, tasks, D):
    worker_map = {w.id: w for w in workers}
    task_map = {t.id: t for t in tasks}

    if action_task_id == -1: return -D # Low payoff for invalid task

    # Check for conflicts (duplicate task assignments) within the joint action
    non_dummy_actions = [tid for tid in joint_action_si_tuple if tid != -1]
    if len(non_dummy_actions) != len(set(non_dummy_actions)):
        # Conflict detected
        return 0.0

    # No conflict, calculate distance-based payoff for worker_id
    worker_pos = worker_map[worker_id].pos
    task_object = task_map.get(action_task_id, None) # Get the Task object

    # Check if the task object was found
    if task_object is None:
        print(f"Error in calculate_payoff: Task ID {action_task_id} not found in task_map.")
        return -D # Return low payoff if task ID is invalid

    # Now access the position from the retrieved task_object
    task_pos = task_object.pos

    # Ensure both positions are valid numpy arrays of shape (2,)
    if not isinstance(worker_pos, np.ndarray) or worker_pos.shape != (2,):
         print(f"Error in calculate_payoff: Invalid worker_pos shape for worker {worker_id}: {worker_pos}")
         return -D
    if not isinstance(task_pos, np.ndarray) or task_pos.shape != (2,):
         print(f"Error in calculate_payoff: Invalid task_pos shape for task {action_task_id}: {task_pos}")
         return -D

    distance = np.linalg.norm(worker_pos - task_pos)
    payoff = max(0.0, D - distance) # Ensure non-negative payoff
    return payoff

# --- Algorithm 3: Solve Graphical Game (Core Logic) ---
def solve_graphical_game(workers, tasks, worker_neighbors_nwl, worker_targets_ntl, D):
    n = len(workers)
    worker_ids = sorted([w.id for w in workers]) # Process workers in fixed order: W0, W1, W2
    calculated_distributions = {}
    start_total_time = time.time()
    worker_map = {w.id: w for w in workers}


    print("\n[步骤二 S23] 开始求解分布式图博弈:")
    print(f"  S232: 工人处理顺序: {worker_ids}")

    for i, current_worker_id in enumerate(worker_ids):
        print(f"\n  --- S233: 处理工人 W{current_worker_id} ({i+1}/{n}) ---")
        start_worker_time = time.time()

        nwl_i = worker_neighbors_nwl[current_worker_id]
        # Create the local joint action space S_i for the neighbors
        neighbor_action_sets = []
        valid_neighbors = True
        for neighbor_id in nwl_i:
            ntl = worker_targets_ntl.get(neighbor_id)
            if not ntl:
                print(f"    错误: 邻居 W{neighbor_id} 的 NTL 未找到.")
                valid_neighbors = False; break
            neighbor_action_sets.append(ntl)
        if not valid_neighbors:
             print(f"    跳过工人 W{current_worker_id} 因为邻居缺少 NTL.")
             calculated_distributions[current_worker_id] = {} # Mark as processed, but no dist
             continue

        joint_actions_si = list(product(*neighbor_action_sets))
        num_joint_actions = len(joint_actions_si)
        print(f"    工人 W{current_worker_id}: 邻居集 NWL={nwl_i}")
        print(f"    本地联合动作空间 |S_{current_worker_id}| = {' x '.join(map(str, map(len, neighbor_action_sets)))} = {num_joint_actions}")

        if num_joint_actions == 0:
            print(f"    警告: W{current_worker_id} 联合动作空间为空. 跳过.")
            calculated_distributions[current_worker_id] = {}
            continue
        # Heuristic limit (adjust if needed)
        if num_joint_actions > 5000:
            print(f"    警告: 联合动作空间过大 ({num_joint_actions}). 跳过 LP 求解.")
            # Fallback: Assign prob 1 to the first action? Or closest target?
            fallback_action = tuple(nas[0] for nas in neighbor_action_sets)
            calculated_distributions[current_worker_id] = {fallback_action: 1.0}
            continue


        # --- LP Formulation ---
        si_to_lp_index = {action: idx for idx, action in enumerate(joint_actions_si)}
        lp_index_to_si = {idx: action for idx, action in enumerate(joint_actions_si)}

        # 1. Objective Function Coefficients (minimize -U_i)
        c = np.zeros(num_joint_actions)
        for idx, si_tuple in lp_index_to_si.items():
            worker_idx_in_nwl = nwl_i.index(current_worker_id)
            action_for_current_worker = si_tuple[worker_idx_in_nwl]
            payoff = calculate_payoff(current_worker_id, action_for_current_worker, si_tuple,
                                      worker_neighbors_nwl, worker_targets_ntl,
                                      workers, tasks, D)
            c[idx] = -payoff # Minimize negative payoff

        # --- Constraints ---
        A_ub = []; b_ub = []; A_eq = []; b_eq = []
        # 2. Normalization Constraint (sum p_i(s_i) = 1)
        A_eq.append(np.ones(num_joint_actions))
        b_eq.append(1.0)
        # 3. CE Constraints (Eq. 11)
        for k_idx_in_nwl, neighbor_k_id in enumerate(nwl_i):
            possible_actions_k = worker_targets_ntl[neighbor_k_id]
            if len(possible_actions_k) <= 1: continue # No deviation possible
            for action_a in possible_actions_k:
                for action_a_prime in possible_actions_k:
                    if action_a == action_a_prime: continue
                    row = np.zeros(num_joint_actions)
                    for lp_idx, si_tuple in lp_index_to_si.items():
                        if si_tuple[k_idx_in_nwl] == action_a:
                            deviating_si_list = list(si_tuple); deviating_si_list[k_idx_in_nwl] = action_a_prime; deviating_si_tuple = tuple(deviating_si_list)
                            payoff_deviate = calculate_payoff(neighbor_k_id, action_a_prime, deviating_si_tuple, worker_neighbors_nwl, worker_targets_ntl, workers, tasks, D)
                            payoff_original = calculate_payoff(neighbor_k_id, action_a, si_tuple, worker_neighbors_nwl, worker_targets_ntl, workers, tasks, D)
                            row[lp_idx] = payoff_deviate - payoff_original
                    A_ub.append(row); b_ub.append(0.0)

        # 4. Consistency Constraints (Eq. 14 / Patent Eq 18 logic)
        processed_neighbors = [nid for nid in nwl_i if nid in calculated_distributions and nid != current_worker_id]
        print(f"    S2331: 判断邻居处理状态: {processed_neighbors if processed_neighbors else '无'} 已处理.")
        if not processed_neighbors:
             print("    S2332: 所有邻居未处理, 执行基础 CE LP 求解.")
        else:
             print(f"    S2333: 已处理邻居 B_{current_worker_id} = {processed_neighbors}. 执行带一致性约束的 CE LP 求解.")

        for neighbor_j_id in processed_neighbors:
            print(f"      应用与已处理邻居 W{neighbor_j_id} 的一致性约束.")
            nwl_j = worker_neighbors_nwl.get(neighbor_j_id)
            if not nwl_j: continue
            shared_neighbor_ids = sorted(list(set(nwl_i) & set(nwl_j)))
            if not shared_neighbor_ids: continue
            print(f"        共享邻居集 NWL_{current_worker_id} ∩ NWL_{neighbor_j_id} = {shared_neighbor_ids}")

            p_j_dist = calculated_distributions.get(neighbor_j_id)
            if not p_j_dist: # Neighbor j might have been skipped
                 print(f"      警告: 邻居 W{neighbor_j_id} 没有计算出的分布，跳过其一致性.")
                 continue

            shared_indices_in_i = [nwl_i.index(shared_id) for shared_id in shared_neighbor_ids]
            shared_indices_in_j = [nwl_j.index(shared_id) for shared_id in shared_neighbor_ids]
            # Joint action space over shared neighbors
            shared_action_sets = []
            valid_shared = True
            for shared_id in shared_neighbor_ids:
                 ntl_shared = worker_targets_ntl.get(shared_id)
                 if not ntl_shared: valid_shared=False; break
                 shared_action_sets.append(ntl_shared)
            if not valid_shared: continue
            joint_actions_shared = list(product(*shared_action_sets))

            # For each joint action a_ij over the shared neighbors
            for shared_action_tuple in joint_actions_shared:
                # Calculate the marginal probability from p_j(s_j)
                prob_j_marginal = 0.0
                # Iterate through stored distribution for j
                for sj_tuple, prob in p_j_dist.items():
                     # Check if length matches before indexing
                     if len(sj_tuple) == len(nwl_j):
                        action_subset_j = tuple(sj_tuple[idx] for idx in shared_indices_in_j)
                        if action_subset_j == shared_action_tuple:
                             prob_j_marginal += prob
                     else:
                        # This indicates an issue potentially in p_j_dist storage or nwl_j definition
                        print(f"      错误: 邻居 W{neighbor_j_id} 存储的动作元组长度 {len(sj_tuple)} 与其邻居数 {len(nwl_j)} 不符.")


                # Create the constraint row for p_i(s_i)
                row = np.zeros(num_joint_actions)
                for lp_idx, si_tuple in lp_index_to_si.items():
                    # Check length before indexing
                    if len(si_tuple) == len(nwl_i):
                         action_subset_i = tuple(si_tuple[idx] for idx in shared_indices_in_i)
                         if action_subset_i == shared_action_tuple:
                            row[lp_idx] = 1.0
                    else:
                        print(f"      错误: 当前工人 W{current_worker_id} 的动作元组长度 {len(si_tuple)} 与其邻居数 {len(nwl_i)} 不符.")

                A_eq.append(row)
                b_eq.append(prob_j_marginal)


        # --- Solve LP ---
        bounds = [(0, 1) for _ in range(num_joint_actions)] # Probabilities between 0 and 1
        A_ub_np = np.array(A_ub) if A_ub else None
        b_ub_np = np.array(b_ub) if b_ub else None
        A_eq_np = np.array(A_eq) if A_eq else None
        b_eq_np = np.array(b_eq) if b_eq else None

        print(f"    求解 W{current_worker_id} 的 LP 问题...")
        lp_start_time = time.time()
        # Use 'highs' solver as it's generally robust
        result = linprog(c, A_ub=A_ub_np, b_ub=b_ub_np, A_eq=A_eq_np, b_eq=b_eq_np, bounds=bounds, method='highs')
        lp_end_time = time.time()
        print(f"    LP 求解耗时 {lp_end_time - lp_start_time:.4f} 秒. 状态: {result.message}")

        current_distribution = {}
        if result.success:
            probabilities = result.x
            print(f"    计算得到的 W{current_worker_id} 联合策略概率分布 p_{current_worker_id} (仅显示概率 > 0.01):")
            total_prob_check = 0
            stored_count = 0
            for lp_idx, prob in enumerate(probabilities):
                 si_tuple = lp_index_to_si[lp_idx]
                 if prob > 1e-6: # Store non-negligible probabilities
                    current_distribution[si_tuple] = prob
                    total_prob_check += prob
                 if prob > 0.01: # Print only significant probabilities
                    print(f"      p_{current_worker_id}( {si_tuple} ) = {prob:.4f}")
                    stored_count += 1
            if stored_count == 0 and probabilities.size > 0: # If no prob > 0.01, print the highest one
                 max_idx = np.argmax(probabilities)
                 print(f"      (最高概率) p_{current_worker_id}( {lp_index_to_si[max_idx]} ) = {probabilities[max_idx]:.4f}")
            print(f"    (分布概率总和校验: {total_prob_check:.4f})")

            calculated_distributions[current_worker_id] = current_distribution

        else:
            print(f"    警告: W{current_worker_id} 的 LP 求解失败. Status: {result.status}, Message: {result.message}")
            # Fallback: Assign uniform over first few actions or a simple deterministic strategy?
            fallback_action = tuple(nas[0] for nas in neighbor_action_sets) if num_joint_actions > 0 else ()
            current_distribution = {fallback_action: 1.0} if num_joint_actions > 0 else {}
            calculated_distributions[current_worker_id] = current_distribution
            print(f"      应用备用策略: p_{current_worker_id}( {fallback_action} ) = 1.0")


        end_worker_time = time.time()
        # print(f"    --- W{current_worker_id} 处理耗时: {end_worker_time - start_worker_time:.4f} 秒 ---")

    end_total_time = time.time()
    print(f"\n[步骤二 S23 结束] 全局图博弈求解总耗时: {end_total_time - start_total_time:.4f} 秒")

    return calculated_distributions

# --- Action Selection ---
def select_actions(workers, calculated_distributions, worker_neighbors_nwl, original_I):
    assignments = {} # Maps original_interceptor_id -> assigned_task_id
    processed_interceptors = set()
    worker_map = {w.id: w for w in workers}

    print("\n[最终任务分配] Action Selection:")

    # Iterate through workers, respecting the original interceptor ID
    for worker in sorted(workers, key=lambda w: (w.original_interceptor_id, w.id)): # Process primary workers first
        orig_interceptor_id = worker.original_interceptor_id
        worker_id = worker.id

        # Only calculate marginals and assign for the *first* worker instance of an interceptor
        if orig_interceptor_id in processed_interceptors:
            continue

        print(f"  计算拦截器 I{orig_interceptor_id} (基于工人 W{worker_id}) 的边缘概率:")

        if worker_id not in calculated_distributions or not calculated_distributions[worker_id]:
            print(f"    警告: 工人 W{worker_id} 没有分布. 分配任务 -1.")
            assignments[orig_interceptor_id] = -1
            processed_interceptors.add(orig_interceptor_id)
            continue

        nwl_i = worker_neighbors_nwl.get(worker_id)
        if not nwl_i :
             print(f"    警告: 工人 W{worker_id} 没有邻居列表 NWL. 分配任务 -1.")
             assignments[orig_interceptor_id] = -1
             processed_interceptors.add(orig_interceptor_id); continue

        worker_idx_in_nwl = nwl_i.index(worker_id)
        distribution = calculated_distributions[worker_id]
        marginal_probs = collections.defaultdict(float)

        if not distribution:
             print(f"    警告: 工人 W{worker_id} 分布为空. 分配任务 -1.")
             assignments[orig_interceptor_id] = -1
             processed_interceptors.add(orig_interceptor_id); continue

        # Calculate marginal probability for each action of the current worker
        for si_tuple, prob in distribution.items():
            if len(si_tuple) == len(nwl_i):
                action = si_tuple[worker_idx_in_nwl]
                marginal_probs[action] += prob
            # else: # Should not happen if LP formulation is correct
            #     print(f"    内部错误: 动作元组 {si_tuple} 长度与 NWL_{worker_id} 不符.")

        if not marginal_probs:
             print(f"    警告: 无法计算 W{worker_id} 的边缘概率. 分配任务 -1.")
             assignments[orig_interceptor_id] = -1
             processed_interceptors.add(orig_interceptor_id); continue

        # Normalize just in case of small floating point errors
        total_prob = sum(marginal_probs.values())
        if total_prob < 1e-6:
             print(f"    警告: W{worker_id} 边缘概率和接近零. 分配任务 -1.")
             assignments[orig_interceptor_id] = -1
             processed_interceptors.add(orig_interceptor_id); continue

        action_list = []
        prob_list = []
        print(f"    计算得到的边缘概率:")
        for action, prob in sorted(marginal_probs.items()):
            norm_prob = prob / total_prob
            print(f"      P(W{worker_id} -> T{action}) = {norm_prob:.4f}")
            action_list.append(action)
            prob_list.append(norm_prob)
        prob_list = np.array(prob_list)

        # Check and fix normalization issues
        if not np.isclose(sum(prob_list), 1.0):
            print(f"      警告: 归一化概率和不为 1 (Sum={sum(prob_list)}). 强制再归一化.")
            prob_list /= prob_list.sum()
        if np.any(prob_list < 0):
             print(f"      错误: 出现负概率. 分配任务 -1.")
             assignments[orig_interceptor_id] = -1
             processed_interceptors.add(orig_interceptor_id); continue


        # Sample action based on marginal probability
        # Ensure valid probability distribution for sampling
        if len(action_list) > 0 and len(action_list) == len(prob_list) and np.all(prob_list >= 0) and np.isclose(np.sum(prob_list), 1.0):
             chosen_action = np.random.choice(action_list, p=prob_list)
             print(f"    根据概率采样: I{orig_interceptor_id} 选择 T{chosen_action}")
             assignments[orig_interceptor_id] = chosen_action
        else:
             print(f"    错误: 无效的概率分布用于采样. 分配任务 -1.")
             assignments[orig_interceptor_id] = -1

        processed_interceptors.add(orig_interceptor_id)

    # Ensure all original interceptors have an entry, even if skipped
    for i in range(original_I):
        if i not in assignments:
            assignments[i] = -1 # Assign -1 if no worker corresponding to it was processed successfully

    return assignments


# --- Main Simulation Example ---
if __name__ == "__main__":
    # --- Example Setup from Patent ---
    np.random.seed(42) # Set seed for reproducibility of sampling

    interceptors_pos = [np.array([2.0, 8.0]), np.array([7.0, 7.0])]
    intruders_pos = [np.array([3.0, 3.0]), np.array([8.0, 4.0]), np.array([5.0, 10.0])]

    print("实施例开始")
    print(f"初始拦截器位置 I0={interceptors_pos[0]}, I1={interceptors_pos[1]}")
    print(f"初始入侵者位置 H0={intruders_pos[0]}, H1={intruders_pos[1]}, H2={intruders_pos[2]}")
    print("-" * 30)

    # 1. Construct Worker/Task Lists
    workers, tasks, orig_i, orig_h = construct_list(interceptors_pos, intruders_pos)
    if not workers or not tasks: print("错误: 未能生成工人或任务列表."); exit()
    print("-" * 30)

    # 2. Build Graph and Neighborhoods
    worker_neighbors_nwl, worker_targets_ntl = build_graph_and_neighborhoods(workers, tasks)
    print("-" * 30)

    # 3. Solve Graphical Game
    calculated_distributions = solve_graphical_game(workers, tasks,
                                                  worker_neighbors_nwl,
                                                  worker_targets_ntl, D_PAYOFF)
    print("-" * 30)

    # 4. Select Actions
    final_assignments = select_actions(workers, calculated_distributions, worker_neighbors_nwl, orig_i)
    print("-" * 30)

    # --- Print Final Results for Patent Example ---
    print("\n[最终分配结果]")
    task_id_to_orig_intruder = {t.id: t.original_intruder_id for t in tasks}
    task_id_to_orig_intruder[-1] = -1 # Handle no assignment case
    assigned_pairs = []
    total_payoff = 0; coverage_count = 0
    assigned_task_ids = []

    for interceptor_id in range(orig_i): # Iterate through original interceptor IDs
        task_id = final_assignments.get(interceptor_id, -1) # Get assignment or default to -1
        orig_intruder_id = task_id_to_orig_intruder.get(task_id, "Unknown")
        print(f"  拦截器 I{interceptor_id} ---> 任务 T{task_id} (来自原始入侵者 H{orig_intruder_id})")
        if task_id != -1:
             assigned_pairs.append((interceptor_id, task_id))
             assigned_task_ids.append(task_id)
             # Payoff calculation (for reference, not strictly part of assignment)
             worker = next((w for w in workers if w.original_interceptor_id == interceptor_id), None)
             task = next((t for t in tasks if t.id == task_id), None)
             if worker and task:
                 dist = np.linalg.norm(worker.pos - task.pos)
                 total_payoff += max(0, D_PAYOFF - dist)

    # Check for conflicts
    conflicts = len(assigned_task_ids) != len(set(assigned_task_ids))
    print(f"\n分配结果中是否存在冲突 (同一任务被分配给多个拦截器): {'是' if conflicts else '否'}")

    # Coverage calculation
    unique_assigned_intruders = {task_id_to_orig_intruder.get(tid) for tid in assigned_task_ids if tid != -1}
    coverage_count = len(unique_assigned_intruders)
    print(f"覆盖的原始入侵者数量: {coverage_count} / {orig_h}")
    # print(f"分配的总收益估算: {total_payoff:.2f}") # Payoff not essential for patent example text