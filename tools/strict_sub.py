

def sub(path, old, new, count=1):
    s = open(path).read()
    if s.count(old) != count:
        raise SystemExit(f"{path}: expected {count} match(es) of {old[:60]!r}, found {s.count(old)}")
    open(path, "w").write(s.replace(old, new))
