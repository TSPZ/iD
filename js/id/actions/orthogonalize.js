iD.actions.Orthogonalize = function(wayId) {

    var dist = iD.geo.dist;

    function orthogonalize(way) {
        var points = way.nodes.map(function(n) { return n.loc.slice(); });
        var score = goodness(points), newScore;
        for (var i = 0; i < 1000; i++) {
            points = step(points);
            newScore = goodness(points);
            if (newScore > score) return points;
            else if (newScore < 1.0e-8) break;
            score = newScore;
        }
        return points;
    }

    function getDotp(a, b, c) {
        var p = normalize(psub(a, b), 1),
            q = normalize(psub(c, b), 1);
        return (p[0] * q[0]) + (p[1] * q[1]);
    }

    function getScore(a, b, c) {
        var dotp = getDotp(a, b, c);
        return 2.0 * Math.min(Math.abs(dotp - 1),
            Math.min(Math.abs(dotp), Math.abs(dotp + 1)));
    }

    function unit(a, b) {
        var angle = Math.atan2(b[1] - a[1], b[0] - a[0]);
        return [Math.cos(angle), Math.sin(angle)];
    }

    function goodness(points) {
        var g = 0;
        for (var i = 1; i < points.length - 1; i++) {
            g += getScore(points[i - 1], points[i], points[i + 1]);
        }
        g += getScore(points[points.length - 1], points[0], points[1]) +
            getScore(points[points.length - 2], points[points.length - 1], points[0]);
        return g;
    }

    function padd(a, b) {
        return [a[0] + b[0], a[1] + b[1]];
    }

    function psub(a, b) {
        return [a[0] - b[0], a[1] - b[1]];
    }

    function pmult(a, l) {
        return [a[0] * l, a[1] * l];
    }

    function normalize(a, l) {
        return pmult(unit([0, 0], a), l || 1);
    }

    function step(points) {
        var motions = points.map(function(b, i) {
            var a = points[(i - 1 + points.length) % points.length],
                c = points[(i + 1) % points.length];

            var p = psub(a, b),
                q = psub(c, b);

            var scale = dist([0, 0], p) + dist([0, 0], q);

            p = normalize(p);
            q = normalize(q);

            var dotp = (p[0] * q[0]) + (p[1] * q[1]);

            // nasty hack to deal with almost-straight segments
            // (angle is closer to 180 than to 90/270).
            if (dotp < -0.707106781186547) dotp += 1.0;

            var v = normalize(unit([0, 0], padd(p, q)), 0.1 * dotp * scale);
            return v;
        });
        for (var i = 0; i < motions.length; i++) {
            points[i] = padd(points[i], motions[i]);
        }
        return points;
    }

    function action(graph) {
        var way = graph.fetch(wayId);
        var res = orthogonalize(way);
        if (res) {
            _.forEach(way.nodes, function(node, i) {
                graph = graph.replace(node.move(res[i]));
            });
        }
        return graph;
    }

    action.enabled = function() {
        return true;
    };

    return action;
};
