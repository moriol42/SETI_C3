#import "@local/rapport:0.2.0": rapport

// LTeX: language=fr
#set text(lang: "fr")

#show: rapport.with(
  title: "TP4 C3 Navigation réactive",
  subtitle: none,
  authors: "Mathieu Oriol",
  show_header: false,
)

= I. Télémètre laser - Follow the gap!

#figure(
  image("Figure_1.png"),
  caption: [Affichage de la position du robot (flèche verte) et des données lidar de l'avant du robot, avec en bleu les points à une distance trop proche du robot, et en vert le point objectif vers lequel le robot se dirige]
)

Cette technique ne fonctionne pas très bien, le robot se prend beaucoup trop les murs.

= II. The bubble!

Pour que le robot se prenne moins les murs, on ajoute des bulles. La fonction `add_bubble(cloud_copy, cloud, i, dist)` rapproche les `br` voisins de `i` dans `cloud` à droite et à gauche à la distance `dist`. J'ai d'abord commencé par mettre `br` à une valeur constante, puis $K times arctan(R / "cloud[i]")$, avec $K$ une constante réglable et $R$ environ le rayon du cercle circonscrit du robot. Ce calcul vient du fait que $tan(alpha) = R / d$ (@trigo).

#figure(
  image("trigo.pdf"),
  caption: [Schema du calcul trigonometrique du rayon de bulle]
) <trigo>

J'ai aussi testé d'appliquer plusieurs méthodes pour sélectionner les points sur lesquels appliquer la bulle : le plus proche, ceux à une distance $K$ fois plus faible que le point le plus éloigné, ceux à une distance $K'$ fois plus importante que le point le plus proche et les $n %$ les plus proches. C'est la dernière méthode qui semble le mieux fonctionner.

Pour le choix du point vers lequel le robot se dirige plutôt que de prendre le plus éloigné (du plus grand gap), j'ai pris celui qui est au centre de la plus grande série de points à une distance de plus de $n %$ du plus éloigné (toujours dans le gap) afin d'essayer d'éviter le cas @cas_lim.


#figure(
  image("cas_limite.png"),
  caption: [Un cas limite à éviter]
) <cas_lim>

Le temps pendant lequel le robot avance sans réexécuter l'algorithme "follow the gap" est également variable et j'ai utilisé la fonction tangente hyperbolique pour avoir une fonction bornée de type sigmoïde.

Pour avoir une conduite plus fluide, j'ai ajouté la possibilité de tourner et d'avancer en même temps si l'angle de rotation est faible et qu'il n'y a pas de mur qui nous bloque devant. Pour cela, j'ai utilisé de l'odométrie pour savoir de combien j'avais déjà tourné.

