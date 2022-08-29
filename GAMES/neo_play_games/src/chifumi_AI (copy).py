#!/usr/bin/env python
# -*- coding: utf-8 -*-

# =======================================================================================================
#     ALFRED CHIFUMI PROGRAM - USING MINASI ALGORITHM
#     https://www.areaprog.com/algo/article-380-algorithme-de-minasi-utilisation-de \
#     -l-algorithme-de-minasi-pour-prevoir-le-choix-d-un-joueur-de-self.pierre-self.papier-self.ciseaux
#
#             Ordre des labels
#             =================
#               0: self.ciseaux
#               1: self.papier
#               2: self.pierre
#
#          elpimous12,  Aout 2018
# =======================================================================================================

"""
les majuscules sont les coups du robot.
"""

import random
import numpy as np

# for ROS
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from alfred_talk.srv import Text2Speach

from time import sleep

class Game():

	def __init__(self):

		# the face detection node will recognize the face and will feed it genre in var
		self.genre = "monsieur."

		self.pattern = ""
		self.coups_gagnants = ["Rp","Ps","Sr"] # Rock / Paper / Scisors
		self.nom_des_formes = ['pierre', 'papier', 'ciseaux']
		self.coups_nuls = ["Rr","Pp","Ss"]
		self.dernier_coup = ""
		self.coup_letter = None

		self.loose = 0
		self.win   = 0
		self.nul   = 0

		self.node_name ="alfred_chifumi"
		rospy.init_node(self.node_name)

		# Subscribe to the ar_pose_marker topic to get the image width and height
		rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_roi)

		# alfred voice
		self.client_speak = rospy.ServiceProxy("/say_fr1", Text2Speach)

		# sequences to speak
		self.win_sentences = [', Bravo, vous gagnez,',', Bien joué',', gagné',', bravo',', vous avez gagné, bravo!',", j'ai perdu",', je viens de perdre']
		self.loose_sentences = [', Vouzavez perdu',', vous venez de perdre',', Perdu',', oh, vous avez perdu',", J'ai gagné",", C'est moi qui gagne",", je gagne"]
		self.win_again_sentences = [", j'ai encore perdu",", vous avez encore gagné",", vous venez encore de gagner"]
		self.loose_again_sentences = [", j'ai encore gagné",", vous avez encore perdu",", vous venez encore de perdre"]
		self.nul_sentences = [", pareil, on recommence",", égalité, on recommence",", égalité, recommençons",", pareil, recommençons"]
		self.nul_again_sentences = ["encore un nul. Recommençons", "Encore une égalité, rejouons","égalité, encore. on rejoue"]
		self.stop_sentences = ["Je quitte le jeux. Merci d'avoir joué avec moi, "+self.genre,"Je ferme le jeu et vous remercie d'avoir joué avec moi, "+self.genre]

	# voice function
	def speak_this(self,text): # robot voice
		self.client_speak(str(text))

	# function receives ar_pose_marker message
	def marker_roi(self, msg):
		try:
			for tag in msg.markers:
				tag_value = tag.id
				if tag_value == 12:
					self.coup = self.nom_des_formes[0]
					self.coup_letter = 'r'
				if tag_value == 25:
					self.coup = self.nom_des_formes[1]
					self.coup_letter = 'p'
				if tag_value == 39:
					self.coup = self.nom_des_formes[2]
					self.coup_letter = 's'
				else: 
					self.coup_letter = None
					self.coup = None

		except:
			pass

	def getNextHit(self):
		# variables servant aux statistiques de coups
		self.pierre = 0
		self.papier = 0
		self.ciseaux = 0

		# *** Algorithme de Minasi ***
		for i in reversed(range(len(self.pattern))): 
			self.sequenceAChercher = self.pattern[i:] # a partir de i, jusqu'a la fin
			self.chaineAnalysee = self.pattern[:i] # a partir du début, pendant i caracteres
			self.lstOccurence = []
			for x in range(0, (len(self.pattern)-len(self.sequenceAChercher))):
				if self.pattern[x:(x+len(self.sequenceAChercher))] ==  self.sequenceAChercher: # si séquence trouvée, ajout du tour suivant
					self.lstOccurence.append(self.pattern[x+len(self.sequenceAChercher):(x+len(self.sequenceAChercher)+2)])
			self.occurence = self.lstOccurence # prévision du prochain coup Robot/user!
			if len(self.occurence) == 0:
				break
			else:
				self.papier = 0
				self.pierre = 0
				self.ciseaux = 0

			#cette fonction va parcourir l'ensemble des prévisions de coups, et lire la deuxième valeur(humain) pour chaque coup,
			# puis les possibilités seront incrémentées, afin de ressortir la plus grande probabilité
			for x in range(0,len(self.occurence)):
				if self.occurence[x][1] =='p':
					self.papier+=1
				if self.occurence[x][1] =='r':
					self.pierre+=1
				if self.occurence[x][1] =='s':
					self.ciseaux+=1

		# choix aléatoire du robot si les prédictions sont nulles
		if self.pierre == 0 and self.papier == 0 and self.ciseaux == 0:
			return random.choice([ "P", "R", "S" ])

		# recherche du score maxi dans les 3 choix
		maxi = max(self.pierre, max(self.papier, self.ciseaux))
		self.possibilite = [ "P", "R", "S" ]
		# extraction de la plus grande probabilité
		if (self.pierre != maxi):
			del self.possibilite[self.possibilite.index("P")]
		if (self.ciseaux != maxi):
			del self.possibilite[self.possibilite.index("R")]
		if (self.papier != maxi):
			del self.possibilite[self.possibilite.index("S")]
		choice = [self.possibilite[0], self.possibilite[len(self.possibilite)-1]]
		self.possibilite = random.choice(choice)
		return self.possibilite

	def play(self):
		# initialisation des crédits :
		self.Robot_coins = 10
		self.Player_coins = 10

		# rappel des règles
		#self.speak_this("Nous voici dans le jeu interactif pierre papier ciseaux. Vous me direz à hauteu voix la forme de main souhaitée, et vous meu montrerez la carte correspondant.")
		#self.speak_this("Par exemple, si vous voulez faire ciseaux, vous direz, ciseaux, et vous meu montrerez la carte ciseaux.")

		self.speak_this("Je suis assez doué à ce jeu, alors bonne chance")

		choice1 = ['Nous commençons la partie avec 10 points chacun.','Nous avons tous les deu, di points.','Vous et moi commençons la partie avec 10 points chacun.']
		#self.speak_this(random.choice(choice1))
		self.speak_this("c'est parti")

		sleep(random.uniform(0.2, 1.5))

		while True:

			self.robot = self.getNextHit()

			if self.robot == 'R':
				self.robot_coup = self.nom_des_formes[0]
			if self.robot == 'P':
				self.robot_coup = self.nom_des_formes[1]
			if self.robot == 'S':
				self.robot_coup = self.nom_des_formes[2]

			self.pattern+=str(self.robot)

			# attente publication d'un retour visuel TAG'
			rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

			# player timing
			if not self.coup_letter :
				print('je ne vois rien')
				continue
				
			# le robot lance la partie :
			self.speak_this('un, deu, trois '+self.robot_coup)

			# coup joué par l'humain
			print('le player a joué '+str(self.coup))

			# on alimente la liste des coups joués, avec celui de l'utilisateur ; le robot a déjà joué 
			self.pattern+=self.coup_letter # ex : 'p' for paper

			# alimentation de la 'base de donnée' (ensemble des coups joués)
			self.dernier_coup = self.pattern[-2:]
			print('last pattern = '+str(self.pattern))
			# labels :
			if self.pattern[-2:][0] == 'R':
				R = self.nom_des_formes[0]
			if self.pattern[-2:][0] == 'P':
				R = self.nom_des_formes[1]
			if self.pattern[-2:][0] == 'S':
				R = self.nom_des_formes[2]
			if self.pattern[-2:][1] == 'r':
				P = self.nom_des_formes[0]
			if self.pattern[-2:][1] == 'p':
				P = self.nom_des_formes[1]
			if self.pattern[-2:][1] == 's':
				P = self.nom_des_formes[2]

			# gagne
			if self.dernier_coup in self.coups_gagnants:
				if self.win >= random.randint(1,4):
					self.speak_this(R+" contre "+P+random.choice(self.win_again_sentences))
					self.win = 0
				else:
					self.speak_this(R+" contre "+P+random.choice(self.win_sentences))
					self.win +=1
					self.loose = 0
					self.nul = 0

				self.Player_coins+=1
				self.Robot_coins-=1


			# coup nul
			elif self.dernier_coup in self.coups_nuls:
				if self.nul >= 1:
					self.speak_this(R+" contre "+P+random.choice(self.nul_again_sentences))
					self.nul = 0
				else:
					self.speak_this(random.choice(self.nul_sentences))
					self.nul+=1
					self.win = 0
					self.loose = 0


			# perd
			else:
				if self.loose >= random.randint(1,4):
					self.speak_this(R+" contre "+P+random.choice(self.loose_again_sentences))
					self.loose = 0
				else:
					self.speak_this(R+" contre "+P+random.choice(self.loose_sentences))
					self.loose+=1
					self.win = 0
					self.nul = 0

				self.Robot_coins+=1
				self.Player_coins-=1


			self.pattern+=str(self.robot)

			# Fin de partie :
			if self.Robot_coins == 0:
				self.speak_this("Je me retrouve à zéro points. Vous venez de gagner la partie, bravo.")
				self.speak_this(random.choice(self.stop_sentences))
				break

			if self.Player_coins == 0:
				self.speak_this("Vous n'avez plus de points. j'ai gagné, coul.")
				self.speak_this(random.choice(self.stop_sentences))
				break

			# scores
			print('Robot_coins :'+str(self.Robot_coins))
			print('     Player_coins :'+str(self.Player_coins))
			self.speak_this("j'ai "+str(self.Robot_coins)+" points, et vous avez "+str(self.Player_coins)+" points.")
			sleep(random.uniform(0.0, 2.0))


if __name__ == "__main__":
		a = Game()
		a.play()


