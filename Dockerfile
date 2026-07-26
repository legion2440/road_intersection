# syntax=docker/dockerfile:1.7

FROM rust:1.88-bookworm AS builder

RUN apt-get update \
    && apt-get install --yes --no-install-recommends libsdl2-dev pkg-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

COPY Cargo.toml Cargo.lock build.rs ./
COPY src ./src

RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/build/target \
    cargo build --release --locked \
    && install -D target/release/road_intersection /out/road_intersection

FROM debian:bookworm-slim AS runtime

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        ca-certificates \
        libsdl2-2.0-0 \
        novnc \
        openbox \
        tini \
        websockify \
        x11vnc \
        xvfb \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 10001 app

WORKDIR /app

COPY --from=builder --chown=app:app /out/road_intersection ./road_intersection
COPY --chown=app:app assets ./assets
COPY --chown=app:app docker/entrypoint.sh /usr/local/bin/road-intersection-entrypoint

RUN chmod 0755 /usr/local/bin/road-intersection-entrypoint

USER app

ENV DISPLAY=:99 \
    ROAD_INTERSECTION_FORCE_RENDER_FALLBACK=1 \
    SCREEN_GEOMETRY=1280x900x24 \
    XDG_RUNTIME_DIR=/tmp/road-intersection-runtime

EXPOSE 6080

HEALTHCHECK --interval=10s --timeout=3s --start-period=10s --retries=3 \
    CMD python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:6080/vnc.html', timeout=2)" || exit 1

ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/road-intersection-entrypoint"]
