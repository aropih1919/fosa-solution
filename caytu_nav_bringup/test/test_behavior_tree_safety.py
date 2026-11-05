"""Le BT Nav2 ne doit contenir aucune récupération à mouvement aveugle."""

from pathlib import Path
import xml.etree.ElementTree as ET


def test_le_bt_replanifie_sans_effacer_ni_pivoter_ni_reculer():
    arbre = (Path(__file__).resolve().parents[1] / "behavior_trees"
             / "navigate_bounded_recovery.xml")
    racine = ET.parse(arbre).getroot()
    noms = {element.tag for element in racine.iter()}

    assert "PipelineSequence" in noms
    assert "ComputePathToPose" in noms
    assert "FollowPath" in noms
    assert "ClearEntireCostmap" not in noms
    assert "BackUp" not in noms
    assert "Spin" not in noms


def test_le_bt_ne_definit_aucune_recovery_nav2_imbriquee():
    arbre = (Path(__file__).resolve().parents[1] / "behavior_trees"
             / "navigate_bounded_recovery.xml")
    racine = ET.parse(arbre).getroot()
    assert not any(element.tag == "RecoveryNode" for element in racine.iter())
