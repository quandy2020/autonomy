"""Resource discovery for source checkouts and wheel installations."""
from autosim.runtime import paths


def test_installed_share_resources_are_found(tmp_path,monkeypatch):
    module=tmp_path/'target/autosim/runtime/paths.py'
    module.parent.mkdir(parents=True)
    module.touch()
    share=tmp_path/'target/share/autosim'
    (share/'config').mkdir(parents=True)
    (share/'config/default.yaml').touch()
    monkeypatch.setattr(paths,'__file__',str(module))
    assert paths.project_root()==share


def test_checkout_resources_take_precedence(tmp_path,monkeypatch):
    module=tmp_path/'autosim/runtime/paths.py'
    module.parent.mkdir(parents=True)
    module.touch()
    (tmp_path/'config').mkdir()
    (tmp_path/'config/default.yaml').touch()
    (tmp_path/'share/autosim/config').mkdir(parents=True)
    (tmp_path/'share/autosim/config/default.yaml').touch()
    monkeypatch.setattr(paths,'__file__',str(module))
    assert paths.project_root()==tmp_path
