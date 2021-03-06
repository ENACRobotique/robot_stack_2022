open Graph_compil;;
open State_machine;;

let version = "0.1"
let usage_msg = "graph_compil [<file>]"
let input_file = ref ""
let need_lib = ref false

let speclist =
  [("--lib", Arg.Set need_lib, "Get the statemachine library to be used with auto-generated graphs. Should be called statemachine.py")];;

let anon_fun = fun filename->
  input_file := filename

let rec read_stdin = fun acc ->
  try
    read_stdin (String.cat acc (really_input_string stdin 1))
  with _ -> acc;;

let () =
  Arg.parse speclist anon_fun usage_msg;
  if !need_lib = true then
    Printf.printf "%s" lib
  else
    if !input_file = "" then
      (* Reads from stdin *)
      let input = read_stdin "" in
      let lexbuf = Lexing.from_string (String.concat "" [input; "\n"]) in
      let stm = Parser.main Lexer.token lexbuf in
      Printf.printf "#generated using graph_compil v%s\n%s" version (generate_stm_py stm)
    else
      (* Reads from file *)
      let ic = open_in !input_file in
      let input = really_input_string ic (in_channel_length ic) in
      let () = close_in ic in
      let lexbuf = Lexing.from_string input in
      let stm = Parser.main Lexer.token lexbuf in
      Printf.printf "#generated using graph_compil v%s\n%s" version (generate_stm_py stm)
