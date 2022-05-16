
module MenhirBasics = struct
  
  exception Error
  
  let _eRR =
    fun _s ->
      raise Error
  
  type token = 
    | RCBR
    | RBR
    | LCBR
    | LBR
    | LABEL
    | INIT
    | IDENTIFIER of (
# 5 "parser.mly"
       (string)
# 21 "parser.ml"
  )
    | EQUAL
    | EOF
    | DIGRAPH
    | COMMENT
    | ARROW_LEFT
  
end

include MenhirBasics

# 1 "parser.mly"
  
  open Graph_compil

# 37 "parser.ml"

type ('s, 'r) _menhir_state = 
  | MenhirState05 : ('s _menhir_cell0_IDENTIFIER _menhir_cell0_IDENTIFIER, _menhir_box_main) _menhir_state
    (** State 05.
        Stack shape : IDENTIFIER IDENTIFIER.
        Start symbol: main. *)

  | MenhirState08 : (('s, _menhir_box_main) _menhir_cell1_LCBR, _menhir_box_main) _menhir_state
    (** State 08.
        Stack shape : LCBR.
        Start symbol: main. *)

  | MenhirState09 : (('s, _menhir_box_main) _menhir_cell1_IDENTIFIER, _menhir_box_main) _menhir_state
    (** State 09.
        Stack shape : IDENTIFIER.
        Start symbol: main. *)

  | MenhirState36 : (('s, _menhir_box_main) _menhir_cell1_statement, _menhir_box_main) _menhir_state
    (** State 36.
        Stack shape : statement.
        Start symbol: main. *)

  | MenhirState42 : ('s _menhir_cell0_IDENTIFIER, _menhir_box_main) _menhir_state
    (** State 42.
        Stack shape : IDENTIFIER.
        Start symbol: main. *)


and ('s, 'r) _menhir_cell1_statement = 
  | MenhirCell1_statement of 's * ('s, 'r) _menhir_state * (Graph_compil.statement)

and ('s, 'r) _menhir_cell1_IDENTIFIER = 
  | MenhirCell1_IDENTIFIER of 's * ('s, 'r) _menhir_state * (
# 5 "parser.mly"
       (string)
# 73 "parser.ml"
)

and 's _menhir_cell0_IDENTIFIER = 
  | MenhirCell0_IDENTIFIER of 's * (
# 5 "parser.mly"
       (string)
# 80 "parser.ml"
)

and ('s, 'r) _menhir_cell1_LCBR = 
  | MenhirCell1_LCBR of 's * ('s, 'r) _menhir_state

and _menhir_box_main = 
  | MenhirBox_main of (Graph_compil.statemachine) [@@unboxed]

let _menhir_action_01 =
  fun guard_and_on_transition sta_id stb_id ->
    (
# 50 "parser.mly"
    (Transition([sta_id], stb_id, guard_and_on_transition))
# 94 "parser.ml"
     : (Graph_compil.statement))

let _menhir_action_02 =
  fun guard_and_on_transition id_ls stb_id ->
    (
# 52 "parser.mly"
    (Transition(id_ls, stb_id, guard_and_on_transition))
# 102 "parser.ml"
     : (Graph_compil.statement))

let _menhir_action_03 =
  fun id ->
    (
# 55 "parser.mly"
                  ([id])
# 110 "parser.ml"
     : (string list))

let _menhir_action_04 =
  fun id id_ls ->
    (
# 56 "parser.mly"
                                  (id::id_ls)
# 118 "parser.ml"
     : (string list))

let _menhir_action_05 =
  fun name st_ls ->
    (
# 31 "parser.mly"
                                                                 (StateMachine(None, name, st_ls))
# 126 "parser.ml"
     : (Graph_compil.statemachine))

let _menhir_action_06 =
  fun name ->
    (
# 32 "parser.mly"
                                          (StateMachine(None, name, []))
# 134 "parser.ml"
     : (Graph_compil.statemachine))

let _menhir_action_07 =
  fun init name st_ls ->
    (
# 33 "parser.mly"
                                                                                        (StateMachine(Some init, name, st_ls))
# 142 "parser.ml"
     : (Graph_compil.statemachine))

let _menhir_action_08 =
  fun init name ->
    (
# 34 "parser.mly"
                                                                 (StateMachine(Some init, name, []))
# 150 "parser.ml"
     : (Graph_compil.statemachine))

let _menhir_action_09 =
  fun on_enter_and_on_leave st_id ->
    (
# 46 "parser.mly"
    (State(st_id, on_enter_and_on_leave))
# 158 "parser.ml"
     : (Graph_compil.statement))

let _menhir_action_10 =
  fun nd ->
    (
# 41 "parser.mly"
                      (nd)
# 166 "parser.ml"
     : (Graph_compil.statement))

let _menhir_action_11 =
  fun edge ->
    (
# 42 "parser.mly"
                        (edge)
# 174 "parser.ml"
     : (Graph_compil.statement))

let _menhir_action_12 =
  fun st ->
    (
# 37 "parser.mly"
                 ([st])
# 182 "parser.ml"
     : (Graph_compil.statement list))

let _menhir_action_13 =
  fun st st_ls ->
    (
# 38 "parser.mly"
                                        (st::st_ls)
# 190 "parser.ml"
     : (Graph_compil.statement list))

let _menhir_print_token : token -> string =
  fun _tok ->
    match _tok with
    | ARROW_LEFT ->
        "ARROW_LEFT"
    | COMMENT ->
        "COMMENT"
    | DIGRAPH ->
        "DIGRAPH"
    | EOF ->
        "EOF"
    | EQUAL ->
        "EQUAL"
    | IDENTIFIER _ ->
        "IDENTIFIER"
    | INIT ->
        "INIT"
    | LABEL ->
        "LABEL"
    | LBR ->
        "LBR"
    | LCBR ->
        "LCBR"
    | RBR ->
        "RBR"
    | RCBR ->
        "RCBR"

let _menhir_fail : unit -> 'a =
  fun () ->
    Printf.eprintf "Internal failure -- please contact the parser generator's developers.\n%!";
    assert false

include struct
  
  [@@@ocaml.warning "-4-37-39"]
  
  let rec _menhir_goto_main : type  ttv_stack. ttv_stack -> _ -> _menhir_box_main =
    fun _menhir_stack _v ->
      MenhirBox_main _v
  
  let rec _menhir_run_45 : type  ttv_stack. ttv_stack _menhir_cell0_IDENTIFIER -> _ -> _ -> _ -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v ->
      let _tok = _menhir_lexer _menhir_lexbuf in
      match (_tok : MenhirBasics.token) with
      | EOF ->
          let MenhirCell0_IDENTIFIER (_menhir_stack, name) = _menhir_stack in
          let st_ls = _v in
          let _v = _menhir_action_05 name st_ls in
          _menhir_goto_main _menhir_stack _v
      | _ ->
          _eRR ()
  
  let rec _menhir_run_33 : type  ttv_stack. ttv_stack _menhir_cell0_IDENTIFIER _menhir_cell0_IDENTIFIER -> _ -> _ -> _ -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v ->
      let _tok = _menhir_lexer _menhir_lexbuf in
      match (_tok : MenhirBasics.token) with
      | EOF ->
          let MenhirCell0_IDENTIFIER (_menhir_stack, name) = _menhir_stack in
          let MenhirCell0_IDENTIFIER (_menhir_stack, init) = _menhir_stack in
          let st_ls = _v in
          let _v = _menhir_action_07 init name st_ls in
          _menhir_goto_main _menhir_stack _v
      | _ ->
          _eRR ()
  
  let rec _menhir_goto_statement_list : type  ttv_stack. ttv_stack -> _ -> _ -> _ -> (ttv_stack, _menhir_box_main) _menhir_state -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s ->
      match _menhir_s with
      | MenhirState42 ->
          _menhir_run_45 _menhir_stack _menhir_lexbuf _menhir_lexer _v
      | MenhirState36 ->
          _menhir_run_37 _menhir_stack _menhir_lexbuf _menhir_lexer _v
      | MenhirState05 ->
          _menhir_run_33 _menhir_stack _menhir_lexbuf _menhir_lexer _v
      | _ ->
          _menhir_fail ()
  
  and _menhir_run_37 : type  ttv_stack. (ttv_stack, _menhir_box_main) _menhir_cell1_statement -> _ -> _ -> _ -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v ->
      let MenhirCell1_statement (_menhir_stack, _menhir_s, st) = _menhir_stack in
      let st_ls = _v in
      let _v = _menhir_action_13 st st_ls in
      _menhir_goto_statement_list _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s
  
  let rec _menhir_run_08 : type  ttv_stack. ttv_stack -> _ -> _ -> (ttv_stack, _menhir_box_main) _menhir_state -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _menhir_s ->
      let _menhir_stack = MenhirCell1_LCBR (_menhir_stack, _menhir_s) in
      let _tok = _menhir_lexer _menhir_lexbuf in
      match (_tok : MenhirBasics.token) with
      | IDENTIFIER _v ->
          _menhir_run_09 _menhir_stack _menhir_lexbuf _menhir_lexer _v MenhirState08
      | _ ->
          _eRR ()
  
  and _menhir_run_09 : type  ttv_stack. ttv_stack -> _ -> _ -> _ -> (ttv_stack, _menhir_box_main) _menhir_state -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s ->
      let _tok = _menhir_lexer _menhir_lexbuf in
      match (_tok : MenhirBasics.token) with
      | IDENTIFIER _v_0 ->
          let _menhir_stack = MenhirCell1_IDENTIFIER (_menhir_stack, _menhir_s, _v) in
          _menhir_run_09 _menhir_stack _menhir_lexbuf _menhir_lexer _v_0 MenhirState09
      | RCBR ->
          let id = _v in
          let _v = _menhir_action_03 id in
          _menhir_goto_id_list _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s
      | _ ->
          _eRR ()
  
  and _menhir_goto_id_list : type  ttv_stack. ttv_stack -> _ -> _ -> _ -> (ttv_stack, _menhir_box_main) _menhir_state -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s ->
      match _menhir_s with
      | MenhirState08 ->
          _menhir_run_11 _menhir_stack _menhir_lexbuf _menhir_lexer _v
      | MenhirState09 ->
          _menhir_run_10 _menhir_stack _menhir_lexbuf _menhir_lexer _v
      | _ ->
          _menhir_fail ()
  
  and _menhir_run_11 : type  ttv_stack. (ttv_stack, _menhir_box_main) _menhir_cell1_LCBR -> _ -> _ -> _ -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v ->
      let _tok = _menhir_lexer _menhir_lexbuf in
      match (_tok : MenhirBasics.token) with
      | ARROW_LEFT ->
          let _tok = _menhir_lexer _menhir_lexbuf in
          (match (_tok : MenhirBasics.token) with
          | IDENTIFIER _v_0 ->
              let _tok = _menhir_lexer _menhir_lexbuf in
              (match (_tok : MenhirBasics.token) with
              | LBR ->
                  let _tok = _menhir_lexer _menhir_lexbuf in
                  (match (_tok : MenhirBasics.token) with
                  | LABEL ->
                      let _tok = _menhir_lexer _menhir_lexbuf in
                      (match (_tok : MenhirBasics.token) with
                      | EQUAL ->
                          let _tok = _menhir_lexer _menhir_lexbuf in
                          (match (_tok : MenhirBasics.token) with
                          | IDENTIFIER _v_1 ->
                              let _tok = _menhir_lexer _menhir_lexbuf in
                              (match (_tok : MenhirBasics.token) with
                              | RBR ->
                                  let _tok = _menhir_lexer _menhir_lexbuf in
                                  let MenhirCell1_LCBR (_menhir_stack, _menhir_s) = _menhir_stack in
                                  let (id_ls, guard_and_on_transition, stb_id) = (_v, _v_1, _v_0) in
                                  let _v = _menhir_action_02 guard_and_on_transition id_ls stb_id in
                                  _menhir_goto_edge_statement _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s _tok
                              | _ ->
                                  _eRR ())
                          | _ ->
                              _eRR ())
                      | _ ->
                          _eRR ())
                  | _ ->
                      _eRR ())
              | _ ->
                  _eRR ())
          | _ ->
              _eRR ())
      | _ ->
          _eRR ()
  
  and _menhir_goto_edge_statement : type  ttv_stack. ttv_stack -> _ -> _ -> _ -> (ttv_stack, _menhir_box_main) _menhir_state -> _ -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s _tok ->
      let edge = _v in
      let _v = _menhir_action_11 edge in
      _menhir_goto_statement _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s _tok
  
  and _menhir_goto_statement : type  ttv_stack. ttv_stack -> _ -> _ -> _ -> (ttv_stack, _menhir_box_main) _menhir_state -> _ -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s _tok ->
      match (_tok : MenhirBasics.token) with
      | LCBR ->
          let _menhir_stack = MenhirCell1_statement (_menhir_stack, _menhir_s, _v) in
          _menhir_run_08 _menhir_stack _menhir_lexbuf _menhir_lexer MenhirState36
      | IDENTIFIER _v_0 ->
          let _menhir_stack = MenhirCell1_statement (_menhir_stack, _menhir_s, _v) in
          _menhir_run_20 _menhir_stack _menhir_lexbuf _menhir_lexer _v_0 MenhirState36
      | RCBR ->
          let st = _v in
          let _v = _menhir_action_12 st in
          _menhir_goto_statement_list _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s
      | _ ->
          _eRR ()
  
  and _menhir_run_20 : type  ttv_stack. ttv_stack -> _ -> _ -> _ -> (ttv_stack, _menhir_box_main) _menhir_state -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s ->
      let _tok = _menhir_lexer _menhir_lexbuf in
      match (_tok : MenhirBasics.token) with
      | LBR ->
          let _tok = _menhir_lexer _menhir_lexbuf in
          (match (_tok : MenhirBasics.token) with
          | COMMENT ->
              let _tok = _menhir_lexer _menhir_lexbuf in
              (match (_tok : MenhirBasics.token) with
              | EQUAL ->
                  let _tok = _menhir_lexer _menhir_lexbuf in
                  (match (_tok : MenhirBasics.token) with
                  | IDENTIFIER _v_0 ->
                      let _tok = _menhir_lexer _menhir_lexbuf in
                      (match (_tok : MenhirBasics.token) with
                      | RBR ->
                          let _tok = _menhir_lexer _menhir_lexbuf in
                          let (on_enter_and_on_leave, st_id) = (_v_0, _v) in
                          let _v = _menhir_action_09 on_enter_and_on_leave st_id in
                          let nd = _v in
                          let _v = _menhir_action_10 nd in
                          _menhir_goto_statement _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s _tok
                      | _ ->
                          _eRR ())
                  | _ ->
                      _eRR ())
              | _ ->
                  _eRR ())
          | _ ->
              _eRR ())
      | ARROW_LEFT ->
          let _tok = _menhir_lexer _menhir_lexbuf in
          (match (_tok : MenhirBasics.token) with
          | IDENTIFIER _v_1 ->
              let _tok = _menhir_lexer _menhir_lexbuf in
              (match (_tok : MenhirBasics.token) with
              | LBR ->
                  let _tok = _menhir_lexer _menhir_lexbuf in
                  (match (_tok : MenhirBasics.token) with
                  | LABEL ->
                      let _tok = _menhir_lexer _menhir_lexbuf in
                      (match (_tok : MenhirBasics.token) with
                      | EQUAL ->
                          let _tok = _menhir_lexer _menhir_lexbuf in
                          (match (_tok : MenhirBasics.token) with
                          | IDENTIFIER _v_2 ->
                              let _tok = _menhir_lexer _menhir_lexbuf in
                              (match (_tok : MenhirBasics.token) with
                              | RBR ->
                                  let _tok = _menhir_lexer _menhir_lexbuf in
                                  let (guard_and_on_transition, stb_id, sta_id) = (_v_2, _v_1, _v) in
                                  let _v = _menhir_action_01 guard_and_on_transition sta_id stb_id in
                                  _menhir_goto_edge_statement _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s _tok
                              | _ ->
                                  _eRR ())
                          | _ ->
                              _eRR ())
                      | _ ->
                          _eRR ())
                  | _ ->
                      _eRR ())
              | _ ->
                  _eRR ())
          | _ ->
              _eRR ())
      | _ ->
          _eRR ()
  
  and _menhir_run_10 : type  ttv_stack. (ttv_stack, _menhir_box_main) _menhir_cell1_IDENTIFIER -> _ -> _ -> _ -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer _v ->
      let MenhirCell1_IDENTIFIER (_menhir_stack, _menhir_s, id) = _menhir_stack in
      let id_ls = _v in
      let _v = _menhir_action_04 id id_ls in
      _menhir_goto_id_list _menhir_stack _menhir_lexbuf _menhir_lexer _v _menhir_s
  
  let rec _menhir_run_00 : type  ttv_stack. ttv_stack -> _ -> _ -> _menhir_box_main =
    fun _menhir_stack _menhir_lexbuf _menhir_lexer ->
      let _tok = _menhir_lexer _menhir_lexbuf in
      match (_tok : MenhirBasics.token) with
      | INIT ->
          let _tok = _menhir_lexer _menhir_lexbuf in
          (match (_tok : MenhirBasics.token) with
          | IDENTIFIER _v ->
              let _tok = _menhir_lexer _menhir_lexbuf in
              (match (_tok : MenhirBasics.token) with
              | DIGRAPH ->
                  let _tok = _menhir_lexer _menhir_lexbuf in
                  (match (_tok : MenhirBasics.token) with
                  | IDENTIFIER _v_0 ->
                      let _tok = _menhir_lexer _menhir_lexbuf in
                      (match (_tok : MenhirBasics.token) with
                      | LCBR ->
                          let _tok = _menhir_lexer _menhir_lexbuf in
                          (match (_tok : MenhirBasics.token) with
                          | RCBR ->
                              let _tok = _menhir_lexer _menhir_lexbuf in
                              (match (_tok : MenhirBasics.token) with
                              | EOF ->
                                  let (name, init) = (_v_0, _v) in
                                  let _v = _menhir_action_08 init name in
                                  _menhir_goto_main _menhir_stack _v
                              | _ ->
                                  _eRR ())
                          | LCBR ->
                              let _menhir_stack = MenhirCell0_IDENTIFIER (_menhir_stack, _v) in
                              let _menhir_stack = MenhirCell0_IDENTIFIER (_menhir_stack, _v_0) in
                              _menhir_run_08 _menhir_stack _menhir_lexbuf _menhir_lexer MenhirState05
                          | IDENTIFIER _v_1 ->
                              let _menhir_stack = MenhirCell0_IDENTIFIER (_menhir_stack, _v) in
                              let _menhir_stack = MenhirCell0_IDENTIFIER (_menhir_stack, _v_0) in
                              _menhir_run_20 _menhir_stack _menhir_lexbuf _menhir_lexer _v_1 MenhirState05
                          | _ ->
                              _eRR ())
                      | _ ->
                          _eRR ())
                  | _ ->
                      _eRR ())
              | _ ->
                  _eRR ())
          | _ ->
              _eRR ())
      | DIGRAPH ->
          let _tok = _menhir_lexer _menhir_lexbuf in
          (match (_tok : MenhirBasics.token) with
          | IDENTIFIER _v ->
              let _tok = _menhir_lexer _menhir_lexbuf in
              (match (_tok : MenhirBasics.token) with
              | LCBR ->
                  let _tok = _menhir_lexer _menhir_lexbuf in
                  (match (_tok : MenhirBasics.token) with
                  | RCBR ->
                      let _tok = _menhir_lexer _menhir_lexbuf in
                      (match (_tok : MenhirBasics.token) with
                      | EOF ->
                          let name = _v in
                          let _v = _menhir_action_06 name in
                          _menhir_goto_main _menhir_stack _v
                      | _ ->
                          _eRR ())
                  | LCBR ->
                      let _menhir_stack = MenhirCell0_IDENTIFIER (_menhir_stack, _v) in
                      _menhir_run_08 _menhir_stack _menhir_lexbuf _menhir_lexer MenhirState42
                  | IDENTIFIER _v_2 ->
                      let _menhir_stack = MenhirCell0_IDENTIFIER (_menhir_stack, _v) in
                      _menhir_run_20 _menhir_stack _menhir_lexbuf _menhir_lexer _v_2 MenhirState42
                  | _ ->
                      _eRR ())
              | _ ->
                  _eRR ())
          | _ ->
              _eRR ())
      | _ ->
          _eRR ()
  
end

let main =
  fun _menhir_lexer _menhir_lexbuf ->
    let _menhir_stack = () in
    let MenhirBox_main v = _menhir_run_00 _menhir_stack _menhir_lexbuf _menhir_lexer in
    v
