##----- Importation des Modules -----##
from tkinter import *
from tkinter import Canvas
from Points import Points
from Obstacle import Coin, Obstacle
from Astar_V4_S import *
import math
import time,threading


#Permet de recuperer la liste des Obstacle fournit par l'IHM
trait = 0
cercle_bleu = 0

##----- Créations des Fonctions -----##
def foo():
    print(time.ctime())
    tps1 = time.time()
    threading.Timer(1,foo).start()
    if started:
        w,x,s,l = dessin.coords('area')
        print(w+47,x+47)
        liste_mob[p.get()-2] = Points(w+47,x+47)
        liste_mob[p.get()-1] = math.sqrt((w-s)**2+(x-l)**2)
    
    global trait
    print (trait)
    dessin.delete(trait)
    liste_obstacle = []
    print("RECTANGLE",liste_ca)
    print("CERCLE",liste_cercle)
    print("OBSTACLE START",liste_obstacle)
    
    

    liste_obstacle = Recuperation_IHM(liste_ca,liste_cercle)
    
    #liste_obstacle.append((Rond(Points(100,50),30)))
    #liste_obstacle.append(liste_cercle)
    
    #liste_obstacle = liste_ca.append(liste_cercle)
    
    print("OBSTACLE APRES",liste_obstacle)

    Depart = Points(50,50)
    Arrive = Points(300,200)

    Chemin_Astar(Arrive,Depart,liste_obstacle)
    tps2=time.time()
    print(tps2 - tps1)

def changeY(coordy):
    newY = 859 - coordy
    return newY

def deplacerforme(event):
    global object_id
    if object_id is not None:
        if formeee.get() == 1:
            dessin.coords(object_id, event.x-100, event.y+40, event.x+100, event.y-40)
            liste_rectxy[i.get()-4] = event.x-100
            liste_rectxy[i.get()-3] = event.y+40
            liste_rectxy[i.get()-2] = event.x+100
            liste_rectxy[i.get()-1] = event.y-50
            liste_ca[f.get()-2] = Points(event.x-100,changeY(event.y)+40) #######
            liste_ca[f.get()-1] = Points(event.x+100,changeY(event.y)-50)#######
        elif formeee.get() == 2:
            dessin.coords(object_id, event.x-40, event.y+40, event.x+47, event.y-47)
            liste_carrxy[j.get()-4] = event.x-40
            liste_carrxy[j.get()-3] = event.y+40
            liste_carrxy[j.get()-2] = event.x+47
            liste_carrxy[j.get()-1] = event.y-47
            liste_ca[f.get()-2] = Points(event.x-40,changeY(event.y)+40)#######
            liste_ca[f.get()-1] = Points(event.x+47,changeY(event.y)-47)#######
        elif formeee.get() == 3:
            dessin.coords(object_id, event.x-40, event.y+40, event.x+47, event.y-47)
            liste_circxy[k.get()-4] = event.x-40
            liste_circxy[k.get()-3] = event.y+40
            liste_circxy[k.get()-2] = event.x+47
            liste_circxy[k.get()-1] = event.y-47
            ray = math.sqrt((event.x-(event.x-40))**2+(event.y-(event.y+40))**2)
            liste_cercle[o.get()-2] = Points(event.x,changeY(event.y))
            liste_cercle[o.get()-1] = ray
            dessin.delete(cercle_bleu)
    liste_valid = [*liste_rectxy, *liste_carrxy, *liste_circxy]
    Text.set("")
    for q in range(0,len(liste_valid),4):
        Text.set(Text.get() + "\nP1(" + str(liste_valid[q]) + ";" + str(liste_valid[q+1]) + ") P2(" + str(liste_valid[q+2]) + ";" + str(liste_valid[q+3]) + ")")


def choisirformes(event):
    global debut
    global started
    global fin
    global object_id
    if formeee.get() == 1:
        object_id = dessin.create_rectangle(event.x+100, event.y+40, event.x-100, event.y-50, fill='black', width=3, outline='red')  
        liste_rect.append(object_id)
        liste_rectxy.append(event.x-100)
        liste_rectxy.append(event.y+40)
        liste_rectxy.append(event.x+100)
        liste_rectxy.append(event.y-50)
        liste_ca.append(Points(event.x-100,changeY(event.y)+40))######
        liste_ca.append(Points(event.x+100,changeY(event.y)-50))#####        Text.set(Text.get() + "\nP1(" + str(liste_rectxy[i.get()]) + ";" + str(liste_rectxy[i.get()+1]) + ") P2(" + str(liste_rectxy[i.get()+2]) + ";" + str(liste_rectxy[i.get()+3]) + ")")
        i.set(i.get()+4)
        f.set(f.get()+2)

    elif formeee.get() == 2:
        object_id = dessin.create_rectangle(event.x+40, event.y+40, event.x-47, event.y-47, fill='black', width=3, outline='red')
        liste_carr.append(object_id)
        liste_carrxy.append(event.x-40)
        liste_carrxy.append(event.y+40)
        liste_carrxy.append(event.x+47)
        liste_carrxy.append(event.y-47)
        liste_ca.append(Points(event.x-40,changeY(event.y)+40))#####
        liste_ca.append(Points(event.x+47,changeY(event.y)-47))#####
        Text.set(Text.get() + "\nP1(" + str(liste_carrxy[j.get()]) + ";" + str(liste_carrxy[j.get()+1]) + ") P2(" + str(liste_carrxy[j.get()+2]) + ";" + str(liste_carrxy[j.get()+3]) + ")")
        j.set(j.get()+4)
        f.set(f.get()+2)

    elif formeee.get() == 3:
        object_id = dessin.create_oval(event.x+40, event.y+40, event.x-47, event.y-47, fill='black', width=3, outline='red')
        liste_circ.append(object_id)
        rayon = math.sqrt((event.x-(event.x-40))**2+(event.y-(event.y+40))**2)
        liste_cercle.append(Points(event.x,changeY(event.y)))######
        liste_cercle.append(rayon)
        print(liste_cercle)
        liste_circxy.append(event.x-40)
        liste_circxy.append(event.y+40)
        liste_circxy.append(event.x+47)
        liste_circxy.append(event.y-47)
        Text.set(Text.get() + "\nP1(" + str(liste_circxy[k.get()]) + ";" + str(liste_circxy[k.get()+1]) + ") P2(" + str(liste_circxy[k.get()+2]) + ";" + str(liste_circxy[k.get()+3]) + ")")
        k.set(k.get()+4)
        o.set(o.get()+2)

    elif formeee.get() == 4:
        debut = event.x
        fin = 1288-debut
        A=(event.x+40,event.y-47)
        B=(event.x-47, event.y+40)
        dessin.create_oval(A, B,outline='red',fill = 'blue',width = 3, tag = 'area')
        #dessin.create_rectangle(event.x+40, event.y+40, event.x-47, event.y-47, fill='blue', width=3, outline='black', tag = 'mobile')
        started = True
        p.set(2)
        animate(1)
    else:
        formeee.set(0)

#http://pascal.ortiz.free.fr/contents/tkinter/tkinter/le_canevas
#http://tkinter.fdex.eu/doc/caw.html#Canvas.tag_bind
#https://sites.google.com/site/pythonpasapas/modules/modules-de-la-bibliotheque-standard/tkinter/tkinterafter
#https://sites.google.com/site/pythonpasapas/modules/modules-de-la-bibliotheque-standard/tkinter/tkinter-canvas-move
# LIEN IMPORTANT :
# https://github.com/arimb/PurePursuit

def valid():
    #liste_valid = [*liste_rectxy, *liste_carrxy, *liste_circxy]
    print("CERCLE : ")
    print(liste_cercle)
    print("CARRES")
    print(liste_ca)

def suppr(event):
    global started
    clic=event.x, event.y
    bord = dessin.find_closest(*clic)
    if formeee.get() == 1:
        del liste_rectxy[i.get()-1]
        del liste_rectxy[i.get()-2]
        del liste_rectxy[i.get()-3]
        del liste_rectxy[i.get()-4]
        i.set(i.get()-4)
        del liste_ca[f.get()-1]
        del liste_ca[f.get()-2]
        f.set(f.get()-2)

    elif formeee.get() == 2:
        del liste_carrxy[j.get()-1]
        del liste_carrxy[j.get()-2]
        del liste_carrxy[j.get()-3]
        del liste_carrxy[j.get()-4]
        j.set(j.get()-4)
        del liste_ca[f.get()-1]
        del liste_ca[f.get()-2]
        f.set(f.get()-2)
    elif formeee.get() == 3:
        del liste_circxy[k.get()-1]
        del liste_circxy[k.get()-2]
        del liste_circxy[k.get()-3]
        del liste_circxy[k.get()-4]
        k.set(k.get()-4)
        del liste_cercle[o.get()-1]
        del liste_cercle[o.get()-2]
        o.set(o.get()-2)
    elif formeee.get() == 4:
        started = False         
    dessin.delete(bord)
    liste_valid = [*liste_rectxy, *liste_carrxy, *liste_circxy]
    Text.set("")
    for q in range(0,len(liste_valid),4):
        Text.set(Text.get() + "\nP1(" + str(liste_valid[q]) + ";" + str(liste_valid[q+1]) + ") P2(" + str(liste_valid[q+2]) + ";" + str(liste_valid[q+3]) + ")")


def horizonta():
    formeee.set(4) # MOBILE HORIZONTAL


def animate(v):
    global debut
    global fin
    #tampon = Text.get()
    if started:
        a,b,c,d=dessin.coords("area")
        if a< debut-100 or c> fin:
            v=-v
        #dessin.move("mobile", v, 0)
        dessin.move("area",v,0)
        #liste_mob=dessin.coords("mobile")
        #liste_mob=dessin.coords("mobile")
        #Text.set(tampon + "\nP1(" + str(liste_mob[0]) + ";" + str(liste_mob[1]) + ") P2(" + str(liste_mob[2]) + ";" + str(liste_mob[3]) + ")")
        dessin.after(50, animate, v)
        

def rec():
    formeee.set(1) # RECTANGLE


def car():
    formeee.set(2) # CARRE


def ro():
    formeee.set(3) # CERCLE


def afficher(event): ##AFFICHE COORDONNEES SOURIS TEMPS REEL
    """ Entrées : Un événement de la souris
        Sortie : Affiche en temps réel les coordonnées de la souris dans la zone de texte"""
    abscisse = event.x
    ordonnee = event.y
    message.configure(text="X = {} et Y = {}".format(abscisse, ordonnee))


def Chemin_Astar(Depart,Arrive,liste_obs):
    #-----Tracer du chemin-----##
    #Chemin
    #barriers =[]
    #barriers.append(Rond(Points(50,50),30))
    #barriers.append(Rond(Points(100,75),30))
    #print(barriers)

    #print(graph.barriers[0].centre.x)
    #Multiplier par le rapport X et Y les points pour ensuite avoir un affichage de qualité
    
    
    liste = [Rond(Points(100,50),30)]
    liste.append(Rond(Points(100,50),30))
    liste.append(Rond(Points(150,75),30))
    liste.append(Rectangle(Points(50,50),Points(80,80)))
    
    
    graph = AStarGraph(liste_obs)

    


    result, cost = AStarSearch(Depart, Arrive, graph)
    #print ("route", result)
    print ("cost", cost)

    #print(graph.barriers)
    #plt.plot([v.x for v in result], [v.y for v in result])
    #for barrier in graph.barriers:
    #    plt.plot([v.x for v in barrier], [v.y for v in barrier])

    

    #########

    i=0
    liste_1D = []

    while i < len(result):
        x = result[i][0]*rapport_x
        y = result[i][1]*rapport_y
        #print(x,y)
        
        liste_1D.append(x)
        liste_1D.append(y)
        i=i+1

    liste_barrriers=[]
    global trait 
    trait = dessin.create_line(liste_1D,fill='red',width=5)

    #print("TYPE", type(trait))
    #print("type :", type(dessin.create_line(liste_1D,fill='red',width=5)))

    #dessin.create_line(liste_1D,fill='red',width=5)
    #dessin.create_line(liste_barrriers,fill='pink',width=5)



    #Boucle qui va permettre de traiter l'Affichage des obstacles (en fonction de leurs types)
    #print(type(graph.barriers[0]))
    i = 0
    bobo = 0
    while(i<len(graph.barriers)):
        print("#######")
        print("-----")
        print(graph.barriers[i])
        print("Taille tableau obstacle",len(graph.barriers))
        print("-----")
        print("#######")

        if(type(graph.barriers[i]) == Rond):
            print("ROND",type(graph.barriers[i]))
            #print("bruh")
            (A, B) = cercle(graph.barriers[i].centre,graph.barriers[i].rayon)
            #dessin.create_oval((180*rapport_x,45*rapport_y),(120*rapport_x,105*rapport_y),outline="blue",  width=5) #"light blue"
            #print("#####")
            #print("Centre")
            #print(graph.barriers[i].centre)
            #print("Rayon")
            #print(graph.barriers[i].rayon)

            global cercle_bleu
            cercle_bleu = dessin.create_oval((A[0]*rapport_x,A[1]*rapport_y),(B[0]*rapport_x,B[1]*rapport_y),outline="blue",  width=5) #"light blue"
        if(type(graph.barriers[i])== Rectangle):
            bobo = bobo +1
            print("VALLEUUUUUUUUUUUUUURS",graph.barriers[i])
            rectangle_bleu = dessin.create_rectangle((graph.barriers[i].a.x*rapport_x,graph.barriers[i].a.y*rapport_y),(graph.barriers[i].c.x*rapport_x,graph.barriers[i].c.y*rapport_y),outline="blue",  width=5) #"light blue"
            
        i = i+1
        print("BOBOBOBOBOBBOBBOBBOBOBOBOOB",bobo)
    #dessin.delete(trait)

def cercle(centre,rayon):
    A = (centre.x-rayon,centre.y+rayon)
    B = (centre.x+rayon,centre.y-rayon)
    #print(A,B)
    return (A, B)

def Recuperation_IHM(liste_rectangle,liste_rond):
    liste_obstacle = []
    #for rectangle in liste_rectangle:
    #    liste_obstacle.append(rectangle)
    i = 0
    while(i<len(liste_rond)):
        r = Rond(Points(liste_rond[i][0]*inv_rapport_x,liste_rond[i][1]*inv_rapport_y),liste_rond[i+1]*1/5)
        liste_obstacle.append(r)
        i = i+2

    j = 0
    while(j<len(liste_rectangle)):
        rect = Rectangle(Points(liste_rectangle[j].x*inv_rapport_x,liste_rectangle[j].y*inv_rapport_y),Points(liste_rectangle[j+1].x*inv_rapport_x,liste_rectangle[j+1].y*inv_rapport_y))
        liste_obstacle.append(rect)
        #print(liste_obstacle)
        j= j+2

    return liste_obstacle


def Map_init(liste_obs):
    c = Coin()
    liste_obs.append(c)
    #Rectangle decor (fond)


##----- Création de la fenêtre -----##
object_id = None
debut = None
fin = None
fen = Tk()
fen.title('Tracer dans un canevas')

##----- Création du canevas -----##
dessin = Canvas(fen, width = 1288, height = 859, bg = 'white')
dessin.grid(row = 0, column = 0, columnspan = 2, padx = 3, pady = 3)
dessin2 = Canvas(fen, width = 400, height = 858, bg = 'white')
dessin2.grid(row = 0, column = 40, columnspan = 1, padx = 3, pady = 3)

e1 = Entry(dessin2)
dessin2.create_window(200,480,window=e1, height=600, width=350)
Text = StringVar()

i = IntVar(0)
j = IntVar(0)
k = IntVar(0)
p = IntVar(0)
f = IntVar(0) # VARIABLE CARRE + RECTANGLE
o = IntVar(0) # VARIABLE CERCLE
LabelResultat = Label(fen, textvariable = Text ,fg = 'white', bg ="grey", width=35, height=40)
dessin2.create_window(200,490,window=LabelResultat)
Text.set("")

##----- Création des boutons -----##
formeee = IntVar()

bouton_rectangle = Button(fen, text='Rectangle', command=rec)
bouton_rectangle = dessin2.create_window(75,105, window = bouton_rectangle)

bouton_carre = Button(fen, text='Carré', command=car)
bouton_carre = dessin2.create_window(170,105, window = bouton_carre)

bouton_rond = Button(fen, text='Rond', command=ro)
bouton_rond = dessin2.create_window(245,105, window = bouton_rond)

bouton_valid = Button(fen, text='Confirmer', command=valid)
bouton_valid = dessin2.create_window(340,105, window = bouton_valid)

bouton_hor = Button(fen, text='Horizontal', command=horizonta)
bouton_hor = dessin2.create_window(200,70, window = bouton_hor)


##----- Création du Label -----##
lbl = Label(dessin2, text='',fg='black',bg='black')


##----- Dessiner dans le canevas -----##
# TEXTE
message = Label(fen, text='Souris Temps Réel')
message.grid(row=1, column=0, columnspan=2, padx=0, pady=0, sticky=W+E)

bouton_quitter = Button(fen, text='Quitter', command=fen.destroy)
bouton_quitter.grid(row = 1, column = 1, padx = 3, pady = 3, sticky=E)

# Image 
im = PhotoImage(file = 'Fonctionnelle/carte_projet.png', master=fen)
logo1 = dessin.create_image(644, 430, image = im )

#Texte IHM PLACER OBSTACLES
dessin2.create_text(200, 30, text='Placer Obstacles : ', fill='#000000', font='Arial 18')

##----- Programme principal -----##
dessin.bind('<Motion>', afficher) ## DEPLACEMENT SOURIS TEMPS REEL
dessin.bind('<Button-1>',choisirformes) ## CLIQUE SOURIS APRES CHOIX FORME
dessin.bind('<Button-3>', deplacerforme)
dessin.bind("<Button-2>",suppr)

started = False
# bind tag 

#VARIABLES GLOBALES


liste_valid = [] # AFFICHAGE
liste_rect = []
liste_rectxy = [] # AFFICHAGE
liste_carr = []
liste_carrxy = [] # AFFICHAGE
liste_circ = []
liste_circxy = [] # AFFICHAGE
liste_mob = []
liste_cercle = [] # CONTIENT CENTRE + RAYON
liste_ca = [] # CONTIENT P1 et P2 CARRE + RECTANGLE



rapport_x = (1288/(3000))*10
rapport_y = (859/(2000))*10

inv_rapport_x = 1/rapport_x
inv_rapport_y = 1/rapport_y



foo()
fen.mainloop()                    # Boucle d'attente des événements