# Graph compil by KirrimK

pour compiler, installer ocaml et son package manager, dune,
puis installer menhir et ocamllex

puis, dans le présent dossier, lancer ```dune build```
enfin, lancer le compilateur avec ```./_build/default/main.exe``` suivi du nom de fichier .dot à traiter
ne pas oublier de piper la sortie standard dans un fichier
possible de ne pas mettre de fichier pour entrer le graphe sur l'entrée standard, ou
de mettre le flag --lib pour récupérer une copie de la librairie statemachine à utiliser avec le code généré
