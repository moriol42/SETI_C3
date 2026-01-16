#import "@local/rapport:0.2.0": rapport

// LTeX: language=fr
#set text(lang: "fr")

#show: rapport.with(
  title: "TP2 C3 Odométrie et déplacements",
  subtitle: none,
  authors: "Mathieu Oriol",
  show_header: false,
)

= Localisation

La fonction `draw_pos` permet d'afficher à l'écran la position calculée avec l'odométrie et de la comparer à celle réelle.

#figure(
  image("trace.png", height: 10cm),
  caption: [Position calculé du robot]
)


#figure(
  image("trace_comp.png", height: 10cm),
  caption: [Comparaison entre position calculé du robot et position réelle]
)

La fonction `draw_comp` permet de comparer les composantes (x, y et $theta$) et leur valeur de référence.

//Faire des mouvements busque 

#figure(
  image("composantes.png", height: 10cm),
  caption: [Comparaison entre les composantes (x, y et $theta$) et leur valeur de référence]
)

On voit que les erreurs s'accumulent par exemple à la fin le robot s'est pris un mur ce qui e fait diverger les valeurs.

= Premiers déplacements

La fonction `chgmt_base(p)` permet d'exprimer les coordonnées de `p` dans le repère du robot

= Déplacements automatisés

La fonction `compute_move(p)` permet de calculer la rotation et la translation à effectuer pour se déplacer jusqu'au point `p` défini dans le repère du robot. Et la fonction `goto_rot_trans(rot, trans)` prends des valeurs de rotation et translation et permet de s'y déplacer.

La fonction `goto(p)` combine les fonctions précédentes et permet de se déplacer vers un point dans le repère de la scène. On pourrait donc s'en servir pour faire le tour du labyrinthe, mais les erreurs sur la position sont trop importantes pour cela.
