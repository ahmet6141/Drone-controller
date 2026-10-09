import yaml, sys
d = yaml.safe_load(open('/home/user/Drone-controller/ucav250/data/research/components.yaml'))
def val(x):
    return x.get('value') if isinstance(x, dict) and 'value' in x else x
def walk(node, path):
    if isinstance(node, dict):
        if 'id' in node:
            keys = [k for k in node if any(s in k for s in ('dim', 'mass', 'diameter', 'width', 'model', 'envelope', 'compartment', 'size', 'volume', 'length', 'height', 'mount', 'rated', 'peak', 'opening', 'shock', 'load'))]
            out = {k: val(node[k]) for k in keys}
            print(path, node['id'], out)
        for k, v in node.items():
            walk(v, path + '.' + str(k))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            walk(v, path + f'[{i}]')
walk(d['categories'], 'categories')
