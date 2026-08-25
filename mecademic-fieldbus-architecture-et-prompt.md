# Librairie fieldbus Mecademic — architecture & prompt de scaffold

## Partie 1 — Document d'architecture

### 1. Contexte

- `mecademicpy` pilote aujourd'hui le robot via TCP/IP (protocole texte ASCII), avec une API riche
  (`Robot`, `robot_classes.py`, `mx_robot_def.py`, `robot_initializer.py`).
- Le firmware expose déjà `EnableEtherNetIp` / `EnableProfinet` / `SwitchToEtherCAT` et
  `GetEtherNetIpEnabled` / `GetProfinetEnabled` côté API texte : le robot sait donc déjà se comporter
  en **adapter/target** fieldbus avec des assemblies IN/OUT cycliques.
- Objectif : une nouvelle librairie Python, **indépendante de `mecademicpy`** (aucune dépendance,
  aucun import), qui joue le rôle de **scanner/originator** EtherNet/IP (puis potentiellement
  Profinet, autre) pour piloter le robot. La surface d'API s'inspire du vocabulaire de `mecademicpy`
  (noms de fonctions, esprit général) pour rester familière, mais tout est réimplémenté localement —
  sur un **sous-ensemble** de fonctions, celui que le mapping IO fieldbus supporte réellement
  (activation, home, reset erreur, statut, IO digitales, et mouvement si l'assembly le permet —
  souvent des positions préréglées plutôt qu'une trajectoire continue).
- Contrainte structurante n°1 : **le mapping des bits/mots des assemblies va évoluer** (nouveaux
  champs, nouvelles versions d'assembly). Le code applicatif ne doit jamais dépendre des offsets
  bruts.
- Contrainte structurante n°2 : **indépendance et portabilité**. La librairie doit pouvoir vivre sans
  `mecademicpy` et sans écosystème Python spécifique à Mecademic, car une évolution future possible
  est un portage vers d'autres langages où `mecademicpy` n'existe pas. Concrètement : zéro dépendance
  vers `mecademicpy`, et la logique de mapping IO — la partie la plus précieuse à préserver dans un
  portage — doit rester aussi indépendante que possible de l'implémentation Python (voir note de
  portabilité en section 3).

### 2. Découpage en couches

```
Application
     │  utilise l'API haut niveau (mêmes noms que mecademicpy quand possible)
     ▼
┌─────────────────────────────┐
│ FieldbusRobot (façade)      │  Layer 4 — surface API publique
└─────────────────────────────┘
     │  encode/decode commandes ↔ champs nommés
     ▼
┌─────────────────────────────┐
│ IoMap (vX)                  │  Layer 3 — mapping déclaratif bits/mots ↔ champs logiques
└─────────────────────────────┘
     │  bytes bruts d'assembly IN/OUT
     ▼
┌─────────────────────────────┐
│ FieldbusTransport            │  Layer 2 — abstraction protocole (EIP, Profinet…)
│  (EtherNetIpTransport, …)    │
└─────────────────────────────┘
     │  wrap une lib tierce (à valider : eeip.py, cpppo, pycomm3…)
     ▼
Réseau fieldbus
```

**Pourquoi cette séparation :**

- **Transport isolé** : swapper la lib tierce EtherNet/IP, ou ajouter Profinet plus tard, ne touche
  qu'une classe — jamais la façade ni le code métier.
- **IoMap isolé** : quand le firmware fait évoluer le layout d'assembly, on ajoute une nouvelle
  version d'`IoMap` (versionnée, testable indépendamment) sans toucher au reste. Aucun offset ne doit
  fuiter en dehors de cette couche.
- **Façade familière, mais autonome** : mêmes noms de fonctions que `mecademicpy` quand c'est
  pertinent (facilite la prise en main pour qui connaît déjà l'API TCP/IP du robot), mais dataclasses
  et logique entièrement réimplémentées localement — zéro dépendance vers `mecademicpy`.
- **Mock robot** : comme le contrat `FieldbusTransport` est simple (bytes en entrée/sortie) et que le
  mapping est isolé dans `IoMap`, on peut construire un **simulateur de robot** complet — utile pour
  développer et tester sans matériel, avant même que le layout d'assembly réel soit figé. Voir
  section 3bis.

### 3. Interfaces clés (squelette indicatif)

```python
# transports/base.py
class FieldbusTransport(abc.ABC):
    @abc.abstractmethod
    def connect(self, address: str, **kwargs) -> None: ...
    @abc.abstractmethod
    def disconnect(self) -> None: ...
    @property
    @abc.abstractmethod
    def is_connected(self) -> bool: ...
    @abc.abstractmethod
    def read_input_assembly(self) -> bytes: ...
    @abc.abstractmethod
    def write_output_assembly(self, data: bytes) -> None: ...

# io_map/base.py
class IoMap(abc.ABC):
    """Traduit des bytes d'assembly bruts <-> champs logiques nommés et typés."""
    version: str

    @abc.abstractmethod
    def decode_status(self, raw_input: bytes) -> "RobotStatus": ...
    @abc.abstractmethod
    def encode_move_command(self, *args, **kwargs) -> bytes: ...
    # etc. — un encode/decode par groupe de champs logiques

# robot.py
class FieldbusRobot:
    """Sous-ensemble de l'API mecademicpy.Robot, piloté par fieldbus."""
    def __init__(self, transport: FieldbusTransport, io_map: IoMap): ...
    def Connect(self, address: str) -> None: ...
    def ActivateRobot(self) -> None: ...
    def Home(self) -> None: ...
    def DeactivateRobot(self) -> None: ...
    def ResetError(self) -> None: ...
    def GetStatusRobot(self) -> "RobotStatus": ...
    def SetOutputState(self, *args) -> None: ...
    # ... sous-ensemble mouvement selon ce que l'assembly permet
```

**Note de portabilité :** pour limiter ce qui devra être réécrit en cas de portage futur vers un
autre langage, il est recommandé de traiter le mapping bits/mots comme une **spécification
déclarative** (YAML/JSON versionné) plutôt que comme uniquement du code Python — voir `io_map/spec/`
dans la structure de repo (section 5). La classe `IoMap` en Python devient alors une couche fine qui
charge/valide cette spec, plutôt que le seul dépositaire de la vérité.

### 3bis. Le mock robot (simulateur)

Deux niveaux de mock, pour deux besoins différents :

1. **Mock au niveau transport (unit tests rapides)** — un `FakeTransport(FieldbusTransport)` en
   mémoire, alimenté par des fixtures de bytes préenregistrées ou générées. Sert à tester `IoMap` et
   `FieldbusRobot` en isolation, sans réseau. Rapide, déterministe, adapté à la CI.

2. **Mock robot réseau (tests d'intégration / dev sans matériel)** — un vrai **serveur EtherNet/IP
   simulé** qui écoute sur le réseau (localhost ou LAN) et se comporte comme le robot : répond au
   Register Session / Forward Open, expose des assemblies IN/OUT, et simule un état interne réaliste
   (machine à états : désactivé → activé → homing → idle → en mouvement → erreur, avec des délais
   plausibles). Ça permet de :
   - développer et tester la vraie stack (`EtherNetIpTransport` + `IoMap` + `FieldbusRobot`) de bout
     en bout sans robot physique ni réseau fieldbus réel ;
   - avoir un outil de démo/formation indépendant du matériel ;
   - documenter noir sur blanc le comportement attendu de l'assembly (utile aussi comme référence
     pour un futur portage vers un autre langage).

   Ce mock robot peut être écrit avec `python-ethernetip` côté serveur (elle gère aussi bien
   l'explicite que l'implicite), ou en sockets bruts si on préfère zéro dépendance côté outil de
   test. À ranger dans un module dédié (`mock_robot/`), séparé du code de production mais versionné
   dans le même repo pour rester synchronisé avec `IoMap`.

### 4. Points d'attention techniques

- **Class 1 (implicite, cyclique, UDP) vs Class 3 (explicite, TCP, request/response)** : le pilotage
  temps réel du robot passera par une connexion Class 1 Originator vers l'assembly IN/OUT du robot.
  Beaucoup de libs Python EtherNet/IP populaires (ex. `pycomm3`) sont orientées messagerie explicite /
  tags Logix (Rockwell) et ne gèrent pas nécessairement l'ouverture de connexions Class 1 vers un
  adapter générique tiers. **`python-ethernetip`** (paperwork/python-ethernetip) va dans le bon sens
  pour ce projet : elle expose directement `registerAssembly` (input/output par instance CIP),
  `sendFwdOpenReq`, et un cycle `startIO`/`produce` pensé pour un adapter générique — pas seulement du
  Rockwell. Bon point de départ pour `EtherNetIpTransport`, à confirmer par un petit POC de connexion
  réelle avant de s'y engager complètement (maintenance de la lib, comportement exact du Forward Open
  face au firmware du robot).
- **EtherCAT** n'est pas un protocole socket comme EtherNet/IP (nécessite généralement un maître temps
  réel, souvent hors pur-Python / driver noyau). S'il fait partie de la vision long terme, prévoir
  que ce ne sera probablement pas "juste un `FieldbusTransport` de plus" mais une extension à part.
- **Configuration du mode fieldbus du robot** (`EnableEtherNetIp`, etc., aujourd'hui accessible via le
  canal texte TCP/IP de `mecademicpy`) est traitée comme **hors périmètre** de cette librairie :
  l'utilisateur active le mode fieldbus par un autre moyen (MecaPortal, ou son propre outillage), et
  la librairie fieldbus se contente de piloter le robot une fois ce mode actif. Ça préserve
  l'indépendance totale vis-à-vis de `mecademicpy`.
- **Tests sans robot physique** : voir section 3bis (mock robot) — deux niveaux, du plus léger
  (transport mocké en mémoire) au plus complet (simulateur réseau).

### 5. Structure de repo proposée

```
mecademic_fieldbus/
    __init__.py
    robot.py                  # FieldbusRobot (façade)
    robot_classes.py          # dataclasses propres à la librairie (RobotStatus, etc.), autonomes
    exceptions.py
    transports/
        base.py                # FieldbusTransport (ABC)
        ethernetip.py           # implémentation EtherNet/IP (wrap python-ethernetip)
        # profinet.py           # à venir
    io_map/
        base.py                 # IoMap (ABC)
        v1.py                   # première version du mapping d'assembly
        spec/                    # source de vérité déclarative du mapping (YAML/JSON),
                                 # réutilisable si portage vers un autre langage un jour
mock_robot/
    simulator.py               # état interne du robot simulé (machine à états)
    server.py                  # serveur EtherNet/IP qui expose le simulateur sur le réseau
docs/
examples/
tests/
    fixtures/                  # bytes d'assembly enregistrés, pour les mocks légers
```

### 6. Étapes suggérées

1. Documenter précisément le layout actuel de l'assembly EtherNet/IP du robot (manuel de
   programmation / fichier EDS), versionné par version de firmware.
2. POC rapide de connexion Class 1 avec `python-ethernetip` (`registerAssembly` + `sendFwdOpenReq`)
   pour valider la faisabilité contre le vrai firmware du robot avant de s'y engager.
3. En parallèle, démarrer le mock robot réseau (section 3bis) — utile dès les premières validations
   du mapping, avant même d'avoir un accès continu à du matériel physique.
4. Implémenter `FieldbusTransport` (EtherNet/IP), `IoMap` v1 (+ sa spec déclarative), et une façade
   minimale (Connect / ActivateRobot / Home / DeactivateRobot / ResetError / GetStatusRobot /
   Get-SetOutputState).
5. Étendre au sous-ensemble mouvement selon ce que l'assembly expose.
6. Tests unitaires (transport mocké léger) + tests d'intégration (contre le mock robot réseau), CI
   sans dépendance à un robot physique.

---

## Partie 2 — Prompt prêt à donner à un agent IA (ex. Claude Code) pour scaffolder le repo

```
Contexte :
Je travaille dans le domaine de la robotique industrielle (robots Mecademic). Le robot peut
fonctionner comme adapter/target EtherNet/IP (assemblies IN/OUT cycliques) une fois ce mode
activé sur le robot par un autre outil (hors périmètre de cette tâche). Je veux créer une
librairie Python AUTONOME, "mecademic_fieldbus" (nom provisoire), qui agit comme
scanner/originator EtherNet/IP pour piloter le robot. Cette librairie NE DOIT DÉPENDRE
D'AUCUNE autre librairie propriétaire Mecademic : elle doit rester 100% autonome, car elle
est susceptible d'être portée vers d'autres langages plus tard, dans des environnements où
aucune librairie Python Mecademic n'existe. L'API de surface peut s'inspirer du vocabulaire
d'APIs robot existantes (des noms de fonctions comme ActivateRobot, Home, GetStatusRobot,
DeactivateRobot, ResetError...) pour rester familière à qui connaît déjà ce type de robot,
mais tout le code, y compris les structures de données, doit être réimplémenté localement
dans ce nouveau package, sans aucun import externe propriétaire.

Objectif de cette tâche :
Scaffolder le squelette du repo selon l'architecture en couches suivante (ne pas
implémenter toute la logique métier, mais les interfaces, classes de base, et quelques
implémentations minimales avec TODO clairement marqués) :

1. transports/base.py : classe abstraite FieldbusTransport avec connect(), disconnect(),
   is_connected, read_input_assembly() -> bytes, write_output_assembly(data: bytes).
2. transports/ethernetip.py : EtherNetIpTransport(FieldbusTransport) qui s'appuie sur la
   librairie tierce "python-ethernetip" (paperwork/python-ethernetip sur Codeberg — supporte
   la messagerie explicite ET l'IO implicite/cyclique via registerAssembly + sendFwdOpenReq,
   pensée pour un adapter générique et pas seulement du matériel Rockwell). Isoler
   complètement les appels à cette librairie dans ce fichier pour pouvoir la remplacer plus
   tard sans toucher au reste du code.
3. io_map/base.py : classe abstraite IoMap versionnée (attribut `version: str`), avec des
   méthodes encode_*/decode_* pour traduire bytes bruts <-> champs logiques nommés/typés.
   Aucun offset ou index de bit ne doit être visible en dehors de cette couche.
4. io_map/spec/ : un format déclaratif simple (YAML ou JSON) décrivant le mapping bits/mots
   d'une version d'assembly (nom du champ, offset, taille, type). io_map/v1.py doit charger
   ce fichier plutôt que coder les offsets en dur en Python — objectif : garder cette
   spec réutilisable si le projet est porté vers un autre langage un jour.
5. robot.py : classe FieldbusRobot(transport: FieldbusTransport, io_map: IoMap) exposant au
   minimum : Connect, Disconnect, ActivateRobot, Home, DeactivateRobot, ResetError,
   GetStatusRobot, SetOutputState, GetRtOutputState.
6. robot_classes.py : dataclasses propres à la librairie (ex. RobotStatus,
   RobotSafetyStatus simplifiés) — entièrement autonomes, sans dépendance externe.
7. exceptions.py : hiérarchie d'exceptions propre et autonome (ex.
   FieldbusConnectionError, FieldbusTimeoutError, FieldbusProtocolError).
8. mock_robot/ : un simulateur de robot pour développer et tester sans matériel.
   - simulator.py : classe RobotSimulator qui modélise l'état interne du robot avec une
     machine à états simple (désactivé -> activé -> homing -> idle -> en mouvement ->
     erreur), et expose des méthodes pour appliquer une commande reçue (bytes de l'assembly
     OUT) et produire l'état courant (bytes de l'assembly IN), en réutilisant le même
     io_map/ que le reste du projet.
   - server.py : un serveur qui expose RobotSimulator sur le réseau via EtherNet/IP
     (idéalement avec python-ethernetip côté serveur), pour permettre de tester
     EtherNetIpTransport de bout en bout sans robot physique.
9. tests/ : tests unitaires avec un FakeTransport en mémoire (fixtures de bytes) pour tester
   IoMap et FieldbusRobot en isolation, PLUS des tests d'intégration qui font tourner le
   mock_robot en local et s'y connectent avec le vrai EtherNetIpTransport. Utiliser pytest.
10. examples/ : un exemple minimal (connect, activate, home, move, deactivate, disconnect)
    utilisable aussi bien contre un vrai robot que contre le mock_robot local.
11. pyproject.toml, README.md de base.

Conventions à respecter :
- Python 3.8+ (ajuster si besoin).
- Type hints partout, docstrings complètes et cohérentes dans tout le repo.
- PEP8, imports triés (isort), formatage cohérent (yapf ou black).
- Ne jamais coder en dur un offset de bit/mot en dehors de io_map/ — c'est la règle
  d'abstraction centrale du projet, à respecter strictement même dans les squelettes.
- Aucune dépendance vers une librairie propriétaire Mecademic existante : ce package doit
  rester autonome et installable indépendamment.
- Marquer clairement (# TODO) tout ce qui dépend d'une décision non encore prise (layout
  exact de l'assembly, sous-ensemble de mouvement supporté, paramètres RPI/Forward Open
  exacts attendus par le firmware).

Ce que je NE veux PAS dans cette tâche :
- Ne pas ajouter de dépendance vers une librairie Mecademic existante.
- Ne pas essayer de couvrir toutes les fonctions robot imaginables — rester sur le
  sous-ensemble listé ci-dessus.
- Ne pas ajouter Profinet ou EtherCAT maintenant, juste s'assurer que l'abstraction
  transports/ permette de les ajouter proprement plus tard (un transport = un fichier,
  même interface).
- Le mock_robot (item 8), lui, doit être fonctionnel en local — c'est la partie sur laquelle
  je veux pouvoir itérer et tester tout de suite, même avant d'avoir accès à un vrai robot.
```

---

**Notes pour toi, pas pour l'agent :**
- Les deux décisions ouvertes de la version précédente sont maintenant tranchées : librairie
  100% indépendante de mecademicpy, et `python-ethernetip` comme candidat de départ pour le
  transport EtherNet/IP.
- Reste à documenter précisément le layout d'assembly réel du robot avant de finaliser
  `io_map/v1.py` — en attendant, le mock robot peut tourner sur un mapping placeholder
  cohérent, et tout basculera d'un coup une fois le vrai layout connu (c'est justement
  l'intérêt de la spec déclarative en `io_map/spec/`).
