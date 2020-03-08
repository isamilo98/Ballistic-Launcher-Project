#!/usr/bin/env node

"use strict";

var fs = require('fs');

var INPUT_FILE="build/reports/spotbugs/main.html";

var html = "";
try {
  html = fs.readFileSync(INPUT_FILE, 'utf8');
} catch(e) {
  console.log('Error:', e.stack);
  process.exit(1);
}

html = html.replace(/<style([\s\S]*?)<\/style>/gi, '');
html = html.replace(/<script([\s\S]*?)<\/script>/gi, '');
html = html.replace(/<\/div>/ig, '\n');
html = html.replace(/<\/li>/ig, '\n');
html = html.replace(/<li>/ig, '  *  ');
html = html.replace(/<\/ul>/ig, '\n');
html = html.replace(/<\/p>/ig, '\n');
html = html.replace(/<br\s*[\/]?>/gi, "\n");
html = html.replace(/<[^>]+>/ig, '');
html = html.replace(/\n\s*\n/g, '\n');

console.log("Printing SpotBugs Report summary. For prettier output, run SpotBugs on your " +
    "local machine.\n\n")

console.log(html);

