
(* The type of tokens. *)

type token = 
  | RCBR
  | RBR
  | LCBR
  | LBR
  | LABEL
  | INIT
  | IDENTIFIER of (string)
  | EQUAL
  | EOF
  | DIGRAPH
  | COMMENT
  | ARROW_LEFT

(* This exception is raised by the monolithic API functions. *)

exception Error

(* The monolithic API. *)

val main: (Lexing.lexbuf -> token) -> Lexing.lexbuf -> (Graph_compil.statemachine)
