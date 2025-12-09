import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';

function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="A comprehensive guide to embodied artificial intelligence">
      <main>
        <div style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          minHeight: '80vh',
          padding: '2rem',
          textAlign: 'center'
        }}>
          <div style={{
            marginBottom: '2rem'
          }}>
            <img
              src={useBaseUrl('/img/logo.svg')}
              alt="Physical AI & Humanoid Robotics"
              style={{width: '200px', height: '200px'}}
            />
          </div>

          <h1 style={{color: '#2e8555', fontSize: '2.5rem', marginBottom: '1rem'}}>
            {siteConfig.title}
          </h1>

          <p style={{fontSize: '1.2rem', maxWidth: '600px', marginBottom: '2rem'}}>
            {siteConfig.tagline}
          </p>

          <div style={{display: 'flex', gap: '1rem', flexWrap: 'wrap', justifyContent: 'center'}}>
            <a
              className="button button--primary button--lg"
              href={useBaseUrl('/docs/intro')}
              style={{margin: '0.5rem'}}
            >
              Start Reading
            </a>

            <a
              className="button button--secondary button--lg"
              href={useBaseUrl('/docs')}
              style={{margin: '0.5rem'}}
            >
              Browse Documentation
            </a>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default Home;