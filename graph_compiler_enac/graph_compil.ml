(* Graph_compil.ml *)

type statement = 
  State of string * string
| Transition of string list * string * string

type statemachine = 
  StateMachine of string option * string * (statement list);;

let st_str = fun st ->
  match st with
    State(id, on_enter_and_on_leave) -> Printf.sprintf "State [%s] enter/leave[%s]" id on_enter_and_on_leave
  | Transition(id_ls, idb, guard_and_on_transition) -> Printf.sprintf "Transition [{%s} -> %s] guard/on_transition[%s]" (String.concat ", " id_ls) idb guard_and_on_transition;;

let stm_str = fun stm ->
  match stm with
    StateMachine(init, imports, st_ls) ->
    Printf.sprintf "StateMachine [init %s] [imports %s]{\n\t%s\n}" (match init with Some x -> x | _ -> "None") imports (String.concat "\n\t" (List.map st_str st_ls));;

let generate_state_transition_py = fun st ->
  match st with
    State(id, on_enter_and_on_leave) ->
      begin match String.split_on_char '/' on_enter_and_on_leave with
        enter::[leave] -> 
          Printf.sprintf "%s = State(\"%s\", %s, %s)" id id enter leave
      | [nope] when nope = "" ->
        Printf.sprintf "%s = State(\"%s\")" id id
      | [enter] ->
        Printf.sprintf "%s = State(\"%s\", %s)" id id enter
      | _ -> failwith (Printf.sprintf "Too much in state %s" (st_str st)) end
  | Transition(id_ls, id_dest, guard_and_on_transition) ->
      let name = String.concat "To" [(String.concat "" id_ls); id_dest] in
      let decl = begin match String.split_on_char '/' guard_and_on_transition with
        on_transition::[guard] ->
          Printf.sprintf "%s = Transition(\"%s\", %s, %s, %s)" name name id_dest on_transition guard
      | [nope] when nope = "" ->
          Printf.sprintf "%s = Transition(\"%s\", %s)" name name id_dest
      | [on_transition] -> 
          Printf.sprintf "%s = Transition(\"%s\", %s, %s)" name name id_dest on_transition
      | _ -> failwith (Printf.sprintf "Too much in transition %s" (st_str st)) end in
      let adding_to_states = String.concat "\n" (List.map (fun x -> Printf.sprintf "%s.add_transition(%s)" x name) id_ls) in
      String.concat "\n" [decl; adding_to_states]

let generate_stm_py = fun stm ->
  match stm with
    StateMachine(init, name, st_ls) ->
      let list_states = List.filter (fun x -> match x with State(_, _) -> true | _ -> false) st_ls in
      let list_trs = List.filter (fun x -> match x with State(_, _) -> false | _ -> true) st_ls in
      let rec infer_missing = fun tr_name ls->
          match ls with
            [] -> true
          | a::b -> begin match a with
                      State(name, _) -> if tr_name = name then false else infer_missing tr_name b
                    | _ -> infer_missing tr_name b end in
      let inferred_states = List.map (fun x -> State(x, "")) (List.filter (fun x -> infer_missing x list_states) (List.map (fun x -> match x with Transition(_, dest, _) -> dest|_->"nope") list_trs)) in
      let rec remove_duplicates = fun ls acc ->
        match ls with
          [] -> acc
        | a::b -> if List.mem a acc then remove_duplicates b acc else remove_duplicates b (a::acc) in
      let py_states = List.map generate_state_transition_py (List.concat [list_states; remove_duplicates inferred_states []]) in
      let py_trs = List.map generate_state_transition_py list_trs in
      let all_printed = String.concat "\n" (List.concat [py_states; py_trs]) in
      let py_stm = Printf.sprintf "%s = StateMachine(%s)"
                    name
                      (match init with
                        Some x -> x
                      | None -> begin match List.nth_opt list_states 0 with
                                        Some (State(y, _)) -> y
                                      | _ -> "None" end) in
      Printf.sprintf "from statemachine import State, Transition, StateMachine\n\n%s\n%s" all_printed py_stm