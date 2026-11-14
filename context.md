# Contexte du projet

## Objectif et consignes

- Projet de vision augmentée et de recalage dynamique 2D multi-objets en temps réel.
- Le code demandé doit être en C++ et s'appuyer sur les méthodes pertinentes des supports de cours (`Cours/Lecture1.pdf` et `Cours/Lecture2.pdf`).
- Avancer question par question et fonctionnalité par fonctionnalité : présenter l'architecture puis attendre l'accord avant de coder; après explication du code, attendre les tests et la validation de l'utilisateur avant de poursuivre.
- Chaque fonctionnalité devra avoir un `main` exécutable pour permettre les essais par l'utilisateur, et un dossier `tests` accueillera les fichiers de test.
- Après validation d'une fonctionnalité, documenter son fonctionnement théorique et pratique dans `README.md`.
- L'utilisateur effectue les tests et valide les étapes; il gère lui-même les commits/push.

## État initial du dépôt

- Branche `main`, dépôt propre au début de la tâche.
- Aucun code C++ ni configuration de build n'est présent.
- `README.md` ne contient pas encore de documentation utile; `context.md` était vide.
- Les données sont dans `Data/images/` et `Data/videos/`.
- Le jeu contient 12 images de référence de voitures et 12 vidéos associées : Toyota, BMW, Honda, Suzuki, Peugeot, Volkswagen, Ford, Hyundai, Mercedes, Range Rover, Citroën et Nissan.
- Les images inspectées sont des photographies de voitures, et non des logos.
- Plusieurs images ont une extension qui ne correspond pas à leur signature réelle (dont trois fichiers WebP nommés `.png`/`.jpg` et quelques JPEG nommés `.png`). Le chargeur OpenCV est utilisé car il identifie le format à partir du contenu; les fichiers qui ne seraient pas décodables produiront une erreur explicite.

## Énoncé et cours

- Titre du projet : « Vision augmentée et recalage dynamique 2D multi-objets en temps réel ».
- La question 1 demande d'identifier, localiser et réaliser le tracking de l'ensemble des images de référence.
- La question 2 concerne l'augmentation/recalage vidéo et reste hors du périmètre immédiat.
- Les objectifs visibles dans le cours portent notamment sur la détection, la reconnaissance et le tracking en temps réel, ainsi que le tracking par marqueurs et par caractéristiques naturelles. Les détails algorithmiques des supports devront être vérifiés avant de figer l'implémentation.

## Question 1 — architecture approuvée et implémentation initiale

L'utilisateur a approuvé l'approche classique OpenCV avec appariement de caractéristiques, homographie/RANSAC et KLT. Les supports de cours montrent les objectifs de tracking par caractéristiques naturelles et, dans Lecture1, une application sans marqueur d'identification/tracking fondée sur des points caractéristiques et un appariement rapide.

Fichiers créés pour cette fonctionnalité :

- `CMakeLists.txt` : cible C++11 `q1_tracking`, dépendante d'OpenCV 4.
- `include/reference_tracker.hpp` et `src/reference_tracker.cpp` : extraction ORB, appariement Hamming avec test de ratio, homographie RANSAC pour la localisation, flot optique pyramidal KLT entre les re-détections périodiques.
- `tests/q1/main.cpp` : exécutable d'essai qui charge toutes les images du dossier donné, lit une vidéo choisie, dessine le contour et affiche le nom de chaque référence suivie. Échap ou `q` ferme la fenêtre.

La question 1 est implémentée, mais pas encore validée par l'utilisateur ni exécutée sur les vidéos. Dans l'environnement de travail, GCC MinGW 6.3 est disponible, mais CMake et les en-têtes/bibliothèques C++ d'OpenCV ne sont pas installés; une compilation locale n'a donc pas été possible. `git diff --check` est le contrôle disponible à ce stade. Le `README.md` sera rédigé après la validation de cette fonctionnalité, comme demandé.