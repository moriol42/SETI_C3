#import "@local/rapport:0.2.0": rapport

// LTeX: language=fr
#set text(lang: "fr")

#show: rapport.with(
  title: "C3 TP7 : Apprentissage supervisé pour les réseaux de neurones artificiels",
  subtitle: none,
  authors: "Mathieu Oriol",
)

= Le perceptron

== Exercices de classification

#figure(
  ```py
  # liste d'entrées possibles pour l'opérateur logique
  data = [[0, 0], [0, 1], [1, 0], [1, 1]]

  # labels correspondants selon liste d'entrées
  ORlabels = [0, 1, 1, 1]

  # Création du perceptron
  # operateurOR = Perceptron(max_iter=40, tol=1e-3)

  # Entrainement
  operateurOR.fit(data, ORlabels)

  # Verification et affichae du score
  print("OR", operateurOR.predict(data) == ORlabels)
  print("Score OR:", operateurOR.score(data, ORlabels))
  ```,
  caption: [Code d'entrainement d'un perceptron pour l'opérateur OR],
)

Les prédictions des opérateur OR et AND sont correctes, alors que celles du XOR ne le sont pas car il est impossible de modéliser un XOR avec un seul perceptron.

Le score est un réel entre 0 et 1 qui permet d'avoir la qualité des résultats du perceptron. Il vaut 1 pour OR et AND car les sorties prédites sont conformes à celle attendues alors qu'il est de 0.5 pour le XOR, qui n'a que la moitié de sorties correctes.


== Exercice d'OU-Exclusif comme un classifieur

#figure(
  ```py
  classifier = MLPClassifier(max_iter=10000, hidden_layer_sizes=(2,), activation="tanh")
  classifier.fit(data, XORlabels)
  print(classifier.predict(data) == XORlabels)
  print(classifier.score(data, XORlabels))
  print(classifier.coefs_)
  print(classifier.intercepts_)
  ```,
  caption: [Code XOR comme un classifieur],
)

Il y a convergence, mais le score est très variable entre les itérations, il est parfois de 1, parfois de 0.5. Les résultats ne sont pas les mêmes d'un entraînement à l'autre.

#figure(
  ```py
  coefs_ = [array([[-3.81181941,  4.63826166],
         [ 4.08563311, -4.54074203]]), array([[-5.42548456],
         [-6.18073457]])]
  intercepts_ = [array([1.75892196, 2.62004432]), array([5.78815416])]
  ```,
  caption: [Poinds du classificateur XOR dont le score est 1],
)

== Exercice d'OU-Exclusif comme une régression

#figure(
  ```py
  regressor = MLPRegressor(
      max_iter=10000, hidden_layer_sizes=(2, 2), activation="tanh", solver="lbfgs"
  )
  regressor.fit(data, XORlabels)
  print(data, regressor.predict(data), XORlabels)
  print(np.isclose(regressor.predict(data), XORlabels, atol=1e-3))
  print(regressor.predict(data) - XORlabels)
  print(regressor.score(data, XORlabels))
  print(regressor.coefs_)
  print(regressor.intercepts_)
  ```,
  caption: [Code XOR comme une régression],
)

Les valeurs de sortie sont proches de celles attendues, mais pas exactement 0 ou 1 (d'où l'utilisation de `np.isclose` pour la vérification et le score aux alentours de 0.9999998263616505 et non 1).


== Exercice d'application robotique

V1 1 tour dans chaque sens,
v2 plrs dans le meme
