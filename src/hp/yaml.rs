use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq)]
pub enum YamlValue {
    Map(BTreeMap<String, YamlValue>),
    List(Vec<YamlValue>),
    Number(f64),
    Bool(bool),
    String(String),
}

#[derive(Debug, Clone)]
struct LineEntry {
    indent: usize,
    key: String,
    value: Option<String>,
}

pub fn parse_document(text: &str) -> Result<YamlValue, String> {
    let entries = lex_lines(text)?;
    let mut index = 0usize;
    let map = parse_map(&entries, &mut index, 0)?;
    if index != entries.len() {
        return Err("trailing YAML entries remained unparsed".to_string());
    }
    Ok(YamlValue::Map(map))
}

fn lex_lines(text: &str) -> Result<Vec<LineEntry>, String> {
    let mut entries = Vec::new();
    for (line_index, raw_line) in text.lines().enumerate() {
        let stripped = strip_comments(raw_line);
        if stripped.trim().is_empty() {
            continue;
        }
        let indent = stripped.chars().take_while(|ch| *ch == ' ').count();
        if indent % 2 != 0 {
            return Err(format!("line {} uses odd indentation", line_index + 1));
        }
        let body = stripped[indent..].trim_end();
        let Some((key, rest)) = body.split_once(':') else {
            return Err(format!("line {} is missing ':'", line_index + 1));
        };
        let key = key.trim();
        if key.is_empty() {
            return Err(format!("line {} has an empty key", line_index + 1));
        }
        let value = if rest.trim().is_empty() {
            None
        } else {
            Some(rest.trim().to_string())
        };
        entries.push(LineEntry {
            indent,
            key: key.to_string(),
            value,
        });
    }
    Ok(entries)
}

fn parse_map(
    entries: &[LineEntry],
    index: &mut usize,
    indent: usize,
) -> Result<BTreeMap<String, YamlValue>, String> {
    let mut map = BTreeMap::new();
    while *index < entries.len() {
        let entry = &entries[*index];
        if entry.indent < indent {
            break;
        }
        if entry.indent > indent {
            return Err(format!("unexpected indentation before key '{}'", entry.key));
        }

        *index += 1;
        let value = if let Some(raw) = &entry.value {
            parse_scalar_or_list(raw)?
        } else {
            parse_map(entries, index, indent + 2).map(YamlValue::Map)?
        };
        map.insert(entry.key.clone(), value);
    }
    Ok(map)
}

fn parse_scalar_or_list(raw: &str) -> Result<YamlValue, String> {
    let trimmed = raw.trim();
    if trimmed.starts_with('[') {
        return parse_inline_list(trimmed);
    }
    if trimmed.eq_ignore_ascii_case("true") {
        return Ok(YamlValue::Bool(true));
    }
    if trimmed.eq_ignore_ascii_case("false") {
        return Ok(YamlValue::Bool(false));
    }
    if let Ok(value) = trimmed.parse::<f64>() {
        return Ok(YamlValue::Number(value));
    }
    Ok(YamlValue::String(unquote(trimmed)))
}

fn parse_inline_list(raw: &str) -> Result<YamlValue, String> {
    if !raw.ends_with(']') {
        return Err(format!("inline list is not closed: {raw}"));
    }
    let inner = &raw[1..raw.len() - 1];
    if inner.trim().is_empty() {
        return Ok(YamlValue::List(Vec::new()));
    }
    let mut items = Vec::new();
    let mut current = String::new();
    let mut in_quotes = false;
    for ch in inner.chars() {
        match ch {
            '"' => {
                in_quotes = !in_quotes;
                current.push(ch);
            }
            ',' if !in_quotes => {
                items.push(parse_scalar_or_list(current.trim())?);
                current.clear();
            }
            _ => current.push(ch),
        }
    }
    if !current.trim().is_empty() {
        items.push(parse_scalar_or_list(current.trim())?);
    }
    Ok(YamlValue::List(items))
}

fn strip_comments(line: &str) -> String {
    let mut in_quotes = false;
    let mut out = String::new();
    for ch in line.chars() {
        match ch {
            '"' => {
                in_quotes = !in_quotes;
                out.push(ch);
            }
            '#' if !in_quotes => break,
            _ => out.push(ch),
        }
    }
    out
}

fn unquote(raw: &str) -> String {
    if raw.starts_with('"') && raw.ends_with('"') && raw.len() >= 2 {
        raw[1..raw.len() - 1].to_string()
    } else {
        raw.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::{YamlValue, parse_document};

    #[test]
    fn parses_nested_maps_and_inline_lists() {
        let doc = r#"
output:
  dir: "dist/hp"
sweep:
  rpm_points: [1500, 2500, 3500]
  write_pv: true
"#;
        let parsed = parse_document(doc).expect("document should parse");
        let YamlValue::Map(root) = parsed else {
            panic!("root should be a map");
        };
        let output = root.get("output").expect("output map");
        let sweep = root.get("sweep").expect("sweep map");
        assert!(matches!(output, YamlValue::Map(_)));
        assert!(matches!(sweep, YamlValue::Map(_)));
    }
}
