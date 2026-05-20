const http = require('node:http');
const https = require('node:https');
const { HttpsProxyAgent } = require('https-proxy-agent');

const PORT = Number(process.env.CHATPILOT_LLM_PROXY_PORT || 8787);
const UPSTREAM = 'https://api.openai.com/v1/chat/completions';
const UPSTREAM_PROXY =
  process.env.CHATPILOT_UPSTREAM_PROXY ||
  process.env.HTTPS_PROXY ||
  process.env.HTTP_PROXY ||
  '';

function writeJson(res, status, body) {
  res.writeHead(status, {
    'Access-Control-Allow-Origin': '*',
    'Access-Control-Allow-Headers': 'authorization, content-type',
    'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
    'Content-Type': 'application/json; charset=utf-8',
  });
  res.end(JSON.stringify(body));
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    const chunks = [];
    req.on('data', (chunk) => chunks.push(chunk));
    req.on('end', () => resolve(Buffer.concat(chunks)));
    req.on('error', reject);
  });
}

async function proxyChatCompletion(req, res) {
  const body = await readBody(req);
  const clientAuth = req.headers.authorization || '';
  const envKey = process.env.OPENAI_API_KEY || process.env.EXPO_PUBLIC_LLM_API_KEY || '';
  const authorization = envKey ? `Bearer ${envKey}` : clientAuth;

  if (!authorization || authorization === 'Bearer ') {
    writeJson(res, 401, {
      error: 'Missing API key. Paste an OpenAI API key in ChatPilot, or set OPENAI_API_KEY before starting the proxy.',
    });
    return;
  }

  const upstream = new URL(UPSTREAM);
  const upstreamReq = https.request(
    {
      hostname: upstream.hostname,
      path: upstream.pathname,
      method: 'POST',
      agent: UPSTREAM_PROXY ? new HttpsProxyAgent(UPSTREAM_PROXY) : undefined,
      headers: {
        Authorization: authorization,
        'Content-Type': 'application/json',
        'Content-Length': body.length,
      },
    },
    (upstreamRes) => {
      res.writeHead(upstreamRes.statusCode || 502, {
        'Access-Control-Allow-Origin': '*',
        'Access-Control-Allow-Headers': 'authorization, content-type',
        'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
        'Content-Type': upstreamRes.headers['content-type'] || 'application/json; charset=utf-8',
      });
      upstreamRes.pipe(res);
    },
  );

  upstreamReq.on('error', (error) => {
    writeJson(res, 502, { error: `OpenAI proxy failed: ${error.message}` });
  });

  upstreamReq.write(body);
  upstreamReq.end();
}

const server = http.createServer(async (req, res) => {
  if (req.method === 'OPTIONS') {
    writeJson(res, 204, {});
    return;
  }

  if (req.method === 'GET' && req.url === '/health') {
    writeJson(res, 200, { ok: true, upstream: UPSTREAM });
    return;
  }

  if (req.method === 'POST' && req.url === '/v1/chat/completions') {
    try {
      await proxyChatCompletion(req, res);
    } catch (error) {
      writeJson(res, 500, { error: error.message });
    }
    return;
  }

  writeJson(res, 404, { error: 'Not found' });
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`ChatPilot LLM proxy listening on http://0.0.0.0:${PORT}`);
  console.log(UPSTREAM_PROXY ? `Using upstream proxy: ${UPSTREAM_PROXY}` : 'Using direct upstream connection');
});
