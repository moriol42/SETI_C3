#import "@local/rapport:0.2.0": rapport

// LTeX: language=fr
#set text(lang: "fr")

#show: rapport.with(
  title: "C3 TP5 : Introduction du modèle du réseau neuronal",
  subtitle: none,
  authors: "Mathieu Oriol",
)

= Introduction au perceptron

== Perceptron : implantation d'une porte logique

#figure(
  table(
    columns: 4,
    table.header([*Opérateur*], [$w_0$], [$w_1$], [$w_2$]),

    [*ET*], [-1.5], [ 1], [1],
    [*OU*], [-0.5], [ 1], [1],
  ),
  caption: [Table des poids à utiliser pour les opérateur ET et OU]
)

Il n'est pas possible de représenter la fonction XOR avec un seul perceptron car la fonction n'est pas séparable linéairement (Il faut au moins 2 droites).

== Exercice d'application

Il faut d'abord s'assurer que les valeurs des capteurs sont de type tout ou rien.

```py
def get_prox_back():
    return np.array(
        [
            1 if distanceSensors[5].getValue() > 0 else 0,
            1 if distanceSensors[6].getValue() else 0,
        ]
    )


robot_speed = 2 * perceptron(
    w_and, get_prox_back()
)
```


== Perceptron analogique

On utilise la fonction tangente hyperbolique comme fonction d'activation et $w_0 = 0, w_1 = 1$

```py
speed = 9 * perceptron(w_analog, np.array([distanceVal[2]]), func_act=math.tanh)
```

Si on ajoute $w_3 = 0.1$, on voit l'effet des poids sur la vitesse comme dans la vidéo `Perceptron_analogique.webm`.
